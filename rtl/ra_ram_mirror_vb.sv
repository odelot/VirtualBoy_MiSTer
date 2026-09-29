// RetroAchievements RAM Mirror for Virtual Boy — Selective Address + RTQuery (Tier 1)
//
// Each VBlank, reads a list of specific rcheevos addresses from DDRAM
// (written by ARM), fetches byte values from WRAM BRAM (dedicated port B)
// or from the cartridge SRAM DDR3 shadow, and writes them back to DDRAM
// for the ARM to read. Between VBlanks it serves the realtime-query
// mailbox so the ARM can resolve cache misses live (smart cache).
//
// Memory routing (rcheevos address → source):
//   $00000-$0FFFF → System RAM / WRAM  (BRAM port B, dedicated to RA)
//   $10000-$1FFFF → Cartridge RAM     (DDR3 shadow at 0x30000000, coherent:
//                                      every CPU/P1 SRAM write also commits
//                                      to the shadow before completing)
//   All other     → return 0x00
//
// Cartridge RAM byte mapping follows what the CPU observes at
// 0x06000000 + n (the rcheevos "real address" for offset n). The emu level
// supplies the active CPU address mask and the packed-x8 flag:
//   packed (exact 32 KiB save, HyperFlash32 layout): shadow byte = (n & mask) >> 1
//   container layout:                                shadow byte = (n & mask)
//
// DDRAM layout (at DDRAM_BASE, ARM phys 0x3D000000) — same protocol as
// ra_ram_mirror_gb/sms v0x02:
//   [0x00000] Header:   magic(32) + region_count(8) + flags(8) + 0(16)
//   [0x00008] Frame:    frame_counter(32) + 0(32)
//   [0x00010] Debug:    {ver(8), 0(8), 0(16), timeout_cnt(16), ok_cnt(16)}
//   [0x00018] Debug2:   {0(16), wram_cnt(16), cram_cnt(16), 0(16)}
//   [0x00040] ArmCfg:   ARM-written config byte (bit 0 = rtquery armed)
//
//   [0x40000] AddrReq:  addr_count(32) + request_id(32)       (ARM → FPGA)
//   [0x40008] Addrs:    addr[0](32) + addr[1](32), ...        (2 per 64-bit word)
//
//   [0x48000] ValResp:  response_id(32) + response_frame(32)  (FPGA → ARM)
//   [0x48008] Values:   val[0..7](8b each), val[8..15], ...   (8 per 64-bit word)
//
//   [0x50000] QryCtrl:  {seq(8), num(8), 0(16), resp_seq(8), ...}
//   [0x50008] QryReq:   16 slots × {addr(32), num_bytes(8), 0(24)}
//   [0x50088] QryResp:  16 slots × {value(32), 0(32)}

module ra_ram_mirror_vb #(
	parameter [27:1] DDRAM_BASE = 27'h6800000  // ARM phys 0x3D000000
)(
	input             clk,           // clk_sys (40 MHz)
	input             reset,
	input             vblank,

	// WRAM BRAM port B (dedicated to RA, read-only). Two byte lanes:
	// upper = odd CPU byte addresses, lower = even.
	output reg [14:0] wram_addr,
	input       [7:0] wram_udout,
	input       [7:0] wram_ldout,

	// Cartridge SRAM geometry (from emu; changes only on image mount)
	input             cram_packed,      // cart_sram_packed_x8_w
	input      [23:0] cram_cpu_mask,    // cart_sram_cpu_addr_mask_w

	// DDRAM channel (req pulse / ready pulse protocol). Carries both the
	// RA protocol traffic at DDRAM_BASE and cart-RAM shadow reads at 0.
	output reg [27:1] ddram_addr,
	output reg [63:0] ddram_din,
	output reg        ddram_req,
	output reg        ddram_rnw,     // 1=read, 0=write
	output reg  [7:0] ddram_be,
	input      [63:0] ddram_dout,
	input             ddram_ready,

	// Status
	output reg        active,
	output reg [31:0] dbg_frame_counter
);

// ======================================================================
// Constants (byte offsets converted to [27:1] half-word offsets)
// ======================================================================
localparam [27:1] ADDRLIST_BASE = DDRAM_BASE + 27'h20000;  // byte 0x40000 / 2
localparam [27:1] VALCACHE_BASE = DDRAM_BASE + 27'h24000;  // byte 0x48000 / 2
localparam [12:0] MAX_ADDRS     = 13'd4096;
localparam [15:0] CORE_VERSION  = 16'h0200;   // 2.0 — reported in header, used for User-Agent

// Realtime query mailbox (Tier 1 smart cache) — [27:1] addressing
localparam [27:1] QUERY_CTRL_ADDR = DDRAM_BASE + 27'h28000;  // byte 0x50000 / 2
localparam [27:1] QUERY_REQ_BASE  = DDRAM_BASE + 27'h28004;  // byte 0x50008 / 2
localparam [27:1] QUERY_RESP_BASE = DDRAM_BASE + 27'h28044;  // byte 0x50088 / 2
localparam [27:1] ARM_CFG_ADDR    = DDRAM_BASE + 27'd32;     // byte 0x40 / 2
localparam [7:0]  MAX_RT_QUERIES  = 8'd16;

// Cartridge SRAM shadow lives at the bottom of the DDR window
// (ARM phys 0x30000000 + shadow byte address).
localparam [27:1] SHADOW_BASE = 27'h0000000;

// ======================================================================
// VBlank edge detection
// ======================================================================
reg vblank_prev;
wire vblank_rising = vblank & ~vblank_prev;
always @(posedge clk) vblank_prev <= vblank;

// Sticky vblank flag: a vblank arriving while a query is being served is
// not lost — the batch scan starts as soon as the FSM returns to idle.
reg vblank_pending;
always @(posedge clk) begin
	if (reset)
		vblank_pending <= 1'b0;
	else if (vblank_rising)
		vblank_pending <= 1'b1;
	else if (state == S_IDLE && vblank_pending)
		vblank_pending <= 1'b0;
end

// ======================================================================
// State machine
// ======================================================================
localparam S_IDLE         = 6'd0;
localparam S_WR_BUSY_HDR  = 6'd1;   // Write header with busy=1
localparam S_WAIT_DDR_WR  = 6'd2;   // Wait DDRAM write ready
localparam S_WAIT_DDR_RD  = 6'd3;   // Wait DDRAM read ready
localparam S_READ_HDR     = 6'd4;   // Issue DDRAM read: addr list header
localparam S_PARSE_HDR    = 6'd5;   // Parse addr_count + request_id
localparam S_READ_PAIR    = 6'd6;   // Issue DDRAM read: address pair
localparam S_PARSE_ADDR   = 6'd7;   // Extract address from word
localparam S_DISPATCH     = 6'd8;   // Route to memory source (shared batch/query)
localparam S_FETCH_WRAM   = 6'd9;   // Set WRAM BRAM address
localparam S_WRAM_WAIT    = 6'd10;  // Wait for BRAM address register latch
localparam S_WRAM_READ    = 6'd11;  // Capture WRAM data (after BRAM latency)
localparam S_FETCH_CRAM   = 6'd12;  // Issue DDR shadow read
localparam S_CRAM_LATCH   = 6'd13;  // Select byte from shadow beat
localparam S_STORE_VAL    = 6'd14;  // Store byte in collect buffer
localparam S_FLUSH_BUF    = 6'd15;  // Write collect buffer to DDRAM
localparam S_WRITE_RESP   = 6'd16;  // Write response header
localparam S_WR_HDR0      = 6'd17;  // Write main header (busy=0)
localparam S_WR_HDR1      = 6'd18;  // Write frame counter
localparam S_WR_DBG       = 6'd19;  // Write debug word 1
localparam S_WR_DBG2      = 6'd20;  // Write debug word 2
localparam S_RD_ARMCFG    = 6'd21;  // Initiate read of ARM config word
localparam S_PARSE_ARMCFG = 6'd22;  // Latch rtquery_armed from rd_data[0]
// Realtime query states
localparam S_QRY_PARSE    = 6'd23;
localparam S_QRY_RD_REQ   = 6'd24;
localparam S_QRY_FETCH    = 6'd25;
localparam S_QRY_BYTE     = 6'd26;  // Accumulate fetched byte, loop 1-4 bytes
localparam S_QRY_WR_RESP  = 6'd27;
localparam S_QRY_WR_CTRL  = 6'd28;

reg [5:0] state;
reg [5:0] return_state;  // for shared DDR wait states
reg [5:0] fetch_ret;     // where S_DISPATCH's fetch delivers its byte

reg [31:0] frame_counter;
always @(posedge clk) dbg_frame_counter <= frame_counter;

reg [63:0] rd_data;         // Captured DDRAM read data
reg [31:0] req_count;       // Number of addresses in request
reg [31:0] req_id;          // Request ID from ARM
reg [12:0] addr_idx;        // Current address index
reg [63:0] addr_word;       // Cached DDRAM word (2 addresses)
reg [31:0] cur_addr;        // Current rcheevos address (batch and query)
reg [63:0] collect_buf;
reg  [3:0] collect_cnt;     // 0-8 bytes in buffer
reg [12:0] val_word_idx;    // DDRAM word index for value writes
reg  [7:0] fetch_byte;
reg  [2:0] cram_byte_sel;   // Byte lane within the 64-bit shadow beat

// Debug counters (per frame)
reg [15:0] dbg_ok_cnt;
reg [15:0] dbg_timeout_cnt; // kept for protocol parity; no timeout path here
reg [15:0] dbg_wram_cnt;
reg [15:0] dbg_cram_cnt;

// Realtime query registers
reg  [7:0] qry_request_seq;
reg  [7:0] qry_last_seen_seq;
reg  [7:0] qry_num;
reg  [4:0] qry_idx;
reg  [7:0] qry_num_bytes;
reg [31:0] qry_value;
reg  [2:0] qry_byte_idx;
reg [10:0] qry_poll_timer;
reg        rtquery_armed;   // set by ARM via RA_ARM_CFG_RTQUERY bit

// ======================================================================
// Address translation
// ======================================================================
// System RAM: rcheevos $00000-$0FFFF → WRAM byte (lane = bit 0)
wire        wram_valid = (cur_addr < 32'h10000);

// Cartridge RAM: rcheevos $10000-$1FFFF → DDR3 shadow byte.
// The rcheevos offset n corresponds to CPU address 0x06000000 + n; apply
// the same masking the CPU-facing SRAM wrapper applies, then collapse to
// the backing/shadow byte the CPU read would return.
wire        cram_valid = (cur_addr >= 32'h10000) && (cur_addr < 32'h20000);
wire [23:0] cram_masked = {8'd0, cur_addr[15:0]} & cram_cpu_mask;
wire [23:0] cram_shadow_byte = cram_packed ? {1'b0, cram_masked[23:1]} : cram_masked;

// ======================================================================
// Main state machine
// ======================================================================
always @(posedge clk) begin
	// Default: deassert single-cycle signals
	ddram_req <= 1'b0;

	if (reset) begin
		state             <= S_IDLE;
		active            <= 1'b0;
		frame_counter     <= 32'd0;
		wram_addr         <= 15'd0;
		qry_last_seen_seq <= 8'd0;
		qry_poll_timer    <= 11'd0;
		rtquery_armed     <= 1'b0;
	end
	else begin
		case (state)

		// =============================================================
		// IDLE: wait for VBlank; between VBlanks poll the query mailbox
		// =============================================================
		S_IDLE: begin
			active <= 1'b0;
			if (vblank_pending) begin
				active          <= 1'b1;
				qry_poll_timer  <= 11'd0;
				dbg_ok_cnt      <= 16'd0;
				dbg_timeout_cnt <= 16'd0;
				dbg_wram_cnt    <= 16'd0;
				dbg_cram_cnt    <= 16'd0;
				state           <= S_WR_BUSY_HDR;
			end
			else if (qry_poll_timer < 11'd2000) begin
				qry_poll_timer <= qry_poll_timer + 11'd1;
			end
			else if (rtquery_armed) begin
				qry_poll_timer <= 11'd0;
				ddram_addr   <= QUERY_CTRL_ADDR;
				ddram_rnw    <= 1'b1;
				ddram_be     <= 8'hFF;
				ddram_req    <= 1'b1;
				return_state <= S_QRY_PARSE;
				state        <= S_WAIT_DDR_RD;
			end
			else begin
				qry_poll_timer <= 11'd0;  // rtquery not armed, skip poll
			end
		end

		// =============================================================
		// Write header with busy=1
		// =============================================================
		S_WR_BUSY_HDR: begin
			ddram_addr   <= DDRAM_BASE;
			ddram_din    <= {CORE_VERSION, 8'h01, 8'd0, 32'h52414348}; // "RACH", busy=1
			ddram_be     <= 8'hFF;
			ddram_rnw    <= 1'b0;
			ddram_req    <= 1'b1;
			return_state <= S_READ_HDR;
			state        <= S_WAIT_DDR_WR;
		end

		S_WAIT_DDR_WR: begin
			if (ddram_ready)
				state <= return_state;
		end

		S_WAIT_DDR_RD: begin
			if (ddram_ready) begin
				rd_data <= ddram_dout;
				state   <= return_state;
			end
		end

		// =============================================================
		// Read address list header from DDRAM
		// =============================================================
		S_READ_HDR: begin
			ddram_addr   <= ADDRLIST_BASE;
			ddram_rnw    <= 1'b1;
			ddram_be     <= 8'hFF;
			ddram_req    <= 1'b1;
			return_state <= S_PARSE_HDR;
			state        <= S_WAIT_DDR_RD;
		end

		S_PARSE_HDR: begin
			req_id <= rd_data[63:32];
			if (rd_data[31:0] == 32'd0) begin
				req_count <= 32'd0;
				state     <= S_WRITE_RESP;
			end else begin
				req_count    <= (rd_data[31:0] > {19'd0, MAX_ADDRS}) ?
				                {19'd0, MAX_ADDRS} : rd_data[31:0];
				addr_idx     <= 13'd0;
				collect_cnt  <= 4'd0;
				collect_buf  <= 64'd0;
				val_word_idx <= 13'd0;
				state        <= S_READ_PAIR;
			end
		end

		// =============================================================
		// Read address pair from DDRAM (2 addrs per 64-bit word)
		// =============================================================
		S_READ_PAIR: begin
			ddram_addr   <= ADDRLIST_BASE + 27'd4 + {14'd0, addr_idx[12:1], 2'b00};
			ddram_rnw    <= 1'b1;
			ddram_be     <= 8'hFF;
			ddram_req    <= 1'b1;
			return_state <= S_PARSE_ADDR;
			state        <= S_WAIT_DDR_RD;
		end

		S_PARSE_ADDR: begin
			if (!addr_idx[0]) begin
				addr_word <= rd_data;
				cur_addr  <= rd_data[31:0];
			end else begin
				cur_addr <= addr_word[63:32];
			end
			fetch_ret <= S_STORE_VAL;
			state     <= S_DISPATCH;
		end

		// =============================================================
		// Route to WRAM or Cart RAM (shared by batch scan and queries;
		// delivers fetch_byte to fetch_ret)
		// =============================================================
		S_DISPATCH: begin
			if (wram_valid) begin
				dbg_wram_cnt <= dbg_wram_cnt + 16'd1;
				state <= S_FETCH_WRAM;
			end
			else if (cram_valid) begin
				dbg_cram_cnt <= dbg_cram_cnt + 16'd1;
				state <= S_FETCH_CRAM;
			end
			else begin
				// Unmapped address: return 0
				fetch_byte <= 8'd0;
				state      <= fetch_ret;
			end
		end

		// =============================================================
		// WRAM: dedicated BRAM port B, registered address + unregistered
		// output → data valid two cycles after the address is set.
		// =============================================================
		S_FETCH_WRAM: begin
			wram_addr <= cur_addr[15:1];
			state     <= S_WRAM_WAIT;
		end

		S_WRAM_WAIT: begin
			state <= S_WRAM_READ;
		end

		S_WRAM_READ: begin
			fetch_byte <= cur_addr[0] ? wram_udout : wram_ldout;
			dbg_ok_cnt <= dbg_ok_cnt + 16'd1;
			state      <= fetch_ret;
		end

		// =============================================================
		// Cart RAM: read the coherent DDR3 shadow beat, select the byte
		// =============================================================
		S_FETCH_CRAM: begin
			ddram_addr    <= SHADOW_BASE + {4'd0, cram_shadow_byte[23:1]};
			cram_byte_sel <= cram_shadow_byte[2:0];
			ddram_rnw     <= 1'b1;
			ddram_be      <= 8'hFF;
			ddram_req     <= 1'b1;
			return_state  <= S_CRAM_LATCH;
			state         <= S_WAIT_DDR_RD;
		end

		S_CRAM_LATCH: begin
			fetch_byte <= rd_data[{cram_byte_sel, 3'b000} +: 8];
			dbg_ok_cnt <= dbg_ok_cnt + 16'd1;
			state      <= fetch_ret;
		end

		// =============================================================
		// Store byte in collect buffer (batch path)
		// =============================================================
		S_STORE_VAL: begin
			case (collect_cnt[2:0])
				3'd0: collect_buf[ 7: 0] <= fetch_byte;
				3'd1: collect_buf[15: 8] <= fetch_byte;
				3'd2: collect_buf[23:16] <= fetch_byte;
				3'd3: collect_buf[31:24] <= fetch_byte;
				3'd4: collect_buf[39:32] <= fetch_byte;
				3'd5: collect_buf[47:40] <= fetch_byte;
				3'd6: collect_buf[55:48] <= fetch_byte;
				3'd7: collect_buf[63:56] <= fetch_byte;
			endcase
			collect_cnt <= collect_cnt + 4'd1;
			addr_idx    <= addr_idx + 13'd1;

			if (collect_cnt == 4'd7 || (addr_idx + 13'd1 >= req_count[12:0])) begin
				state <= S_FLUSH_BUF;
			end
			else if (addr_idx[0]) begin
				// Was odd → next is even → need new pair from DDRAM
				state <= S_READ_PAIR;
			end else begin
				// Was even → next is odd → use cached high half
				fetch_ret <= S_STORE_VAL;
				cur_addr  <= addr_word[63:32];
				state     <= S_DISPATCH;
			end
		end

		// =============================================================
		// Flush collect buffer to DDRAM value cache
		// =============================================================
		S_FLUSH_BUF: begin
			ddram_addr <= VALCACHE_BASE + 27'd4 + {14'd0, val_word_idx[12:0], 2'b00};
			ddram_din  <= collect_buf;
			ddram_be   <= (collect_cnt == 4'd8) ? 8'hFF
			             : ((8'd1 << collect_cnt[2:0]) - 8'd1);
			ddram_rnw  <= 1'b0;
			ddram_req  <= 1'b1;
			val_word_idx <= val_word_idx + 13'd1;
			collect_cnt  <= 4'd0;
			collect_buf  <= 64'd0;

			if (addr_idx >= req_count[12:0])
				return_state <= S_WRITE_RESP;
			else if (!addr_idx[0])
				return_state <= S_READ_PAIR;
			else
				return_state <= S_PARSE_ADDR;

			state <= S_WAIT_DDR_WR;
		end

		// =============================================================
		// Write response header: {response_frame, response_id}
		// =============================================================
		S_WRITE_RESP: begin
			ddram_addr   <= VALCACHE_BASE;
			ddram_din    <= {frame_counter + 32'd1, req_id};
			ddram_be     <= 8'hFF;
			ddram_rnw    <= 1'b0;
			ddram_req    <= 1'b1;
			return_state <= S_WR_HDR0;
			state        <= S_WAIT_DDR_WR;
		end

		S_WR_HDR0: begin
			ddram_addr   <= DDRAM_BASE;
			ddram_din    <= {CORE_VERSION, 8'h00, 8'd0, 32'h52414348}; // busy=0
			ddram_be     <= 8'hFF;
			ddram_rnw    <= 1'b0;
			ddram_req    <= 1'b1;
			return_state <= S_WR_HDR1;
			state        <= S_WAIT_DDR_WR;
		end

		S_WR_HDR1: begin
			ddram_addr    <= DDRAM_BASE + 27'd4;
			ddram_din     <= {32'd0, frame_counter + 32'd1};
			ddram_be      <= 8'hFF;
			ddram_rnw     <= 1'b0;
			ddram_req     <= 1'b1;
			frame_counter <= frame_counter + 32'd1;
			return_state  <= S_WR_DBG;
			state         <= S_WAIT_DDR_WR;
		end

		// Debug word 1: version 0x02 in the top byte (byte 0x17) marks
		// rtquery mailbox support for ra_rtquery_supported().
		S_WR_DBG: begin
			ddram_addr   <= DDRAM_BASE + 27'd8;
			ddram_din    <= {8'h02, 8'd0, 16'd0, dbg_timeout_cnt, dbg_ok_cnt};
			ddram_be     <= 8'hFF;
			ddram_rnw    <= 1'b0;
			ddram_req    <= 1'b1;
			return_state <= S_WR_DBG2;
			state        <= S_WAIT_DDR_WR;
		end

		S_WR_DBG2: begin
			ddram_addr   <= DDRAM_BASE + 27'd12;
			ddram_din    <= {16'd0, dbg_wram_cnt, dbg_cram_cnt, 16'd0};
			ddram_be     <= 8'hFF;
			ddram_rnw    <= 1'b0;
			ddram_req    <= 1'b1;
			return_state <= S_RD_ARMCFG;
			state        <= S_WAIT_DDR_WR;
		end

		// Read ARM-written config byte once per VBlank. The ARM sets
		// RA_ARM_CFG_RTQUERY (bit 0) when rtquery is active; it gates the
		// inter-VBlank query mailbox polling.
		S_RD_ARMCFG: begin
			ddram_addr   <= ARM_CFG_ADDR;
			ddram_rnw    <= 1'b1;
			ddram_be     <= 8'hFF;
			ddram_req    <= 1'b1;
			return_state <= S_PARSE_ARMCFG;
			state        <= S_WAIT_DDR_RD;
		end

		S_PARSE_ARMCFG: begin
			rtquery_armed <= rd_data[0];
			state <= S_IDLE;
		end

		// =============================================================
		// Realtime query mailbox
		// =============================================================
		S_QRY_PARSE: begin
			if (rd_data[7:0] != qry_last_seen_seq && rd_data[15:8] != 8'd0) begin
				qry_request_seq <= rd_data[7:0];
				qry_num         <= (rd_data[15:8] > MAX_RT_QUERIES) ?
				                   MAX_RT_QUERIES : rd_data[15:8];
				qry_idx         <= 5'd0;
				state           <= S_QRY_RD_REQ;
			end else begin
				state <= S_IDLE;
			end
		end

		S_QRY_RD_REQ: begin
			ddram_addr   <= QUERY_REQ_BASE + {21'd0, qry_idx[3:0], 2'b00};
			ddram_rnw    <= 1'b1;
			ddram_be     <= 8'hFF;
			ddram_req    <= 1'b1;
			return_state <= S_QRY_FETCH;
			state        <= S_WAIT_DDR_RD;
		end

		// Multi-byte reads (1-4) loop through the shared dispatch, one
		// byte per pass, assembling the value little-endian. Reads that
		// straddle a region boundary route each byte independently.
		S_QRY_FETCH: begin
			cur_addr      <= rd_data[31:0];
			qry_num_bytes <= (rd_data[39:32] == 8'd0) ? 8'd1 :
			                 (rd_data[39:32] > 8'd4)  ? 8'd4 : rd_data[39:32];
			qry_value     <= 32'd0;
			qry_byte_idx  <= 3'd0;
			fetch_ret     <= S_QRY_BYTE;
			state         <= S_DISPATCH;
		end

		S_QRY_BYTE: begin
			qry_value    <= qry_value | ({24'd0, fetch_byte} << {qry_byte_idx, 3'b000});
			qry_byte_idx <= qry_byte_idx + 3'd1;
			if ({5'd0, qry_byte_idx} + 8'd1 >= qry_num_bytes) begin
				state <= S_QRY_WR_RESP;
			end else begin
				cur_addr  <= cur_addr + 32'd1;
				fetch_ret <= S_QRY_BYTE;
				state     <= S_DISPATCH;
			end
		end

		S_QRY_WR_RESP: begin
			ddram_addr   <= QUERY_RESP_BASE + {21'd0, qry_idx[3:0], 2'b00};
			ddram_din    <= {32'd0, qry_value};
			ddram_be     <= 8'hFF;
			ddram_rnw    <= 1'b0;
			ddram_req    <= 1'b1;
			qry_idx      <= qry_idx + 5'd1;
			if ({3'd0, qry_idx} + 8'd1 >= qry_num)
				return_state <= S_QRY_WR_CTRL;
			else
				return_state <= S_QRY_RD_REQ;
			state <= S_WAIT_DDR_WR;
		end

		S_QRY_WR_CTRL: begin
			qry_last_seen_seq <= qry_request_seq;
			ddram_addr   <= QUERY_CTRL_ADDR;
			ddram_din    <= {24'd0, qry_request_seq, 16'd0, qry_num, qry_request_seq};
			ddram_be     <= 8'hFF;
			ddram_rnw    <= 1'b0;
			ddram_req    <= 1'b1;
			return_state <= S_IDLE;
			state        <= S_WAIT_DDR_WR;
		end

		default: state <= S_IDLE;
		endcase
	end
end

endmodule
