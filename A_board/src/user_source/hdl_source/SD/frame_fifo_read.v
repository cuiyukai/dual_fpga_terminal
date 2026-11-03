`timescale 1ns/1ps
module frame_fifo_read
#
(
	parameter MEM_DATA_BITS          = 32,
	parameter ADDR_BITS              = 21,
	parameter BURST_BITS             = 9,
	parameter FIFO_DEPTH             = 512,
	parameter BURST_SIZE             = 128,
	parameter FRAME_WIDTH            = 640
)               
(
	input                            rst,                  
	input                            mem_clk,                    // external memory controller user interface clock
	input							 Sdr_init_done,
	input							 Sdr_init_ref_vld,
    input							 Sdr_busy,
    input							 Sdr_rd_en,
	input							 App_wr_busy,
    output							 O_rd_busy,
	output 							 App_rd_en,
	output  [ADDR_BITS - 1:0]		 App_rd_addr,
	
	input                            read_req,
	output reg                       read_req_ack,               // data read module read request response
	output                           read_finish,                // data read module read request finish
	input[ADDR_BITS - 1:0]           read_addr_0,
	input[ADDR_BITS - 1:0]           read_addr_1,
	input[ADDR_BITS - 1:0]           read_addr_2,
	input[ADDR_BITS - 1:0]           read_addr_3,
	input[1:0]                       read_addr_index,
	input[ADDR_BITS - 1:0]           read_len,                   // data read module read request data length
	output reg                       fifo_aclr,                  // to fifo asynchronous clear
	input[9:0]                      wrusedw,                    // from fifo write used words
	input                            read_flip_en,
	input[9:0]                       read_flip_fold,
	input[ADDR_BITS - 1:0]           read_flip_addr

);
localparam ONE                       = 256'd1;                   //256 bit '1'   you can use ONE[n-1:0] for n bit '1'
localparam ZERO                      = 256'd0;                   //256 bit '0'
//read state machine code
localparam S_IDLE                    = 0;                        //idle state,waiting for frame read
localparam S_ACK                     = 1;                        //read request response
localparam S_CHECK_FIFO              = 2;
localparam S_READ_BURST              = 3;                        //begin a burst read
localparam S_READ_BURST_END          = 4;                        //a burst read complete
localparam S_END                     = 5;                        //a frame of data is read to complete
localparam ROW_WORDS                 = 256;

reg                                  read_req_d0;
reg                                  read_req_d1;                //second
reg                                  read_req_d2;                //third,Why do you need 3 ? Here's the design habit
reg[ADDR_BITS - 1:0]                 read_len_d0;
reg[ADDR_BITS - 1:0]                 read_len_d1;                //second
reg[ADDR_BITS - 1:0]                 read_len_latch;             //lock read data length
reg[ADDR_BITS - 1:0]                 read_cnt;                   //read data counter
reg[3:0]                             state;                      //state machine
reg[1:0]                             read_addr_index_d0;         //synchronize to 'mem_clk' clock domain first
reg[1:0]                             read_addr_index_d1;         //synchronize to 'mem_clk' clock domain second
reg [ADDR_BITS - 1:0]	 App_rd_addr_r;
reg [ADDR_BITS - 1:0]                App_rd_addr_alt;
reg                                  rd_alt_sel;                 //App_rd_addr_r currently points into read_flip_addr
reg [9:0]                            rd_col;                     //column of the next word to read
(* ASYNC_REG = "TRUE" *) reg [10:0]  read_flip_meta;
(* ASYNC_REG = "TRUE" *) reg [10:0]  read_flip_sync;             //second
reg                                  flip_en_latch;
reg [9:0]                            flip_fold_latch;
reg [9:0]                            seg_left;                   //words left before the fold or the line end
reg [8:0]                            blk_left;                   //words left in the current SDRAM row
reg                                  next_alt;                   //bank side of the next burst
reg [BURST_BITS - 1:0]               next_len;                   //length of the next burst
reg [BURST_BITS - 1:0]               burst_len;                  //length of the running burst
wire                                 burst_start;
wire                                 swap_now;                   //registers only, kept off the App_wr_busy path

reg [BURST_BITS - 1:0]				burst_cnt;
wire								rd_burst_finish;
reg App_rd_en_r;
reg App_rd_en_d0;

wire rd_vld;
reg [3:0] rd_delay;

assign App_rd_addr = {App_rd_addr_r[ADDR_BITS - 1:0]};
assign rd_vld = (state == S_READ_BURST && burst_cnt >= burst_len);
assign swap_now = (state == S_CHECK_FIFO) && (next_alt != rd_alt_sel);
assign burst_start = (state == S_CHECK_FIFO) && ~read_req_d2 && (next_alt == rd_alt_sel) &&
                     (wrusedw < (FIFO_DEPTH - BURST_SIZE)) && ~App_wr_busy;

assign O_rd_busy = (state == S_READ_BURST);//读指令期间
//burst_cnt代表发送的读指令
assign rd_burst_finish = (rd_vld && rd_delay == 4'd10);
assign read_finish = (state == S_END) ? 1'b1 : 1'b0;             //read finish at state 'S_END'
assign App_rd_en = App_rd_en_d0;
always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		read_req_d0    <=  1'b0;
		read_req_d1    <=  1'b0;
		read_req_d2    <=  1'b0;
		read_len_d0    <=  ZERO[ADDR_BITS - 1:0];               //equivalent to read_len_d0 <= 0;
		read_len_d1    <=  ZERO[ADDR_BITS - 1:0];               //equivalent to read_len_d1 <= 0;
		read_addr_index_d0 <= 2'b00;
		read_addr_index_d1 <= 2'b00;
	end
	else
	begin
		read_req_d0    <=  read_req;
		read_req_d1    <=  read_req_d0;
		read_req_d2    <=  read_req_d1;     
		read_len_d0    <=  read_len;
		read_len_d1    <=  read_len_d0; 
		read_addr_index_d0 <= read_addr_index;
		read_addr_index_d1 <= read_addr_index_d0;
		
	end 
end

always @(posedge mem_clk or posedge rst)
begin
	if(rst || App_rd_en)begin
        rd_delay <= 4'd0;
    end
    else if(rd_delay < 4'd10)begin
    	rd_delay <= rd_delay + 1'b1;
    end
end
always @(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		burst_cnt <= ZERO[BURST_BITS - 1:0];
		App_rd_addr_r <= ZERO[ADDR_BITS - 1:0];
		App_rd_addr_alt <= ZERO[ADDR_BITS - 1:0];
		rd_alt_sel <= 1'b0;
		rd_col <= 10'd0;
		flip_en_latch <= 1'b0;
		flip_fold_latch <= 10'd0;
		burst_len <= BURST_SIZE[BURST_BITS - 1:0];
		App_rd_en_d0 <= 1'b0;
	end
	else begin
	
		if(state == S_CHECK_FIFO)
			burst_cnt <= ZERO[BURST_BITS - 1:0];
		else if(App_rd_en)
			burst_cnt <= burst_cnt + 1'b1;
		else
			burst_cnt <= burst_cnt;
		//
		if(state == S_ACK)
			begin
				if(read_addr_index_d1 == 2'd0)
					App_rd_addr_r <= read_addr_0;
				else if(read_addr_index_d1 == 2'd1)
					App_rd_addr_r <= read_addr_1;
				else if(read_addr_index_d1 == 2'd2)
					App_rd_addr_r <= read_addr_2;
				else if(read_addr_index_d1 == 2'd3)
					App_rd_addr_r <= read_addr_3;
				App_rd_addr_alt <= read_flip_addr;
				rd_alt_sel <= 1'b0;
				flip_en_latch <= read_flip_sync[10];
				flip_fold_latch <= read_flip_sync[9:0];
			end
		else if(swap_now)
			begin
				App_rd_addr_r <= App_rd_addr_alt;
				App_rd_addr_alt <= App_rd_addr_r;
				rd_alt_sel <= next_alt;
			end
		else if(App_rd_en)
			begin
				App_rd_addr_r <= App_rd_addr_r + 1'b1;
				App_rd_addr_alt <= App_rd_addr_alt + 1'b1;
			end
		//
		if(state == S_ACK)
			rd_col <= 10'd0;
		else if(App_rd_en)
			rd_col <= (rd_col == FRAME_WIDTH - 1) ? 10'd0 : rd_col + 1'b1;
		if(state == S_CHECK_FIFO)
			burst_len <= next_len;
		//
		if(App_rd_en_r && burst_cnt + App_rd_en < burst_len)
			App_rd_en_d0 <= 1'b1;
		else
			App_rd_en_d0 <= 1'b0;
	
	end		
end
always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		read_flip_meta <= 11'd0;
		read_flip_sync <= 11'd0;
		seg_left <= 10'd0;
		blk_left <= 9'd0;
		next_alt <= 1'b0;
		next_len <= BURST_SIZE[BURST_BITS - 1:0];
	end
	else
	begin
		read_flip_meta <= {read_flip_en, read_flip_fold};
		read_flip_sync <= read_flip_meta;
		seg_left <= (rd_col < flip_fold_latch) ? flip_fold_latch - rd_col : FRAME_WIDTH - rd_col;
		blk_left <= ROW_WORDS - App_rd_addr_r[7:0];
		next_alt <= flip_en_latch && (rd_col >= flip_fold_latch);
		if(~flip_en_latch)
			next_len <= BURST_SIZE[BURST_BITS - 1:0];
		else if(seg_left <= blk_left && seg_left <= BURST_SIZE)
			next_len <= seg_left;
		else if(blk_left <= BURST_SIZE)
			next_len <= blk_left;
		else
			next_len <= BURST_SIZE[BURST_BITS - 1:0];
	end
end
always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		state <= S_IDLE;
		read_len_latch <= ZERO[ADDR_BITS - 1:0];
		
		App_rd_en_r <= 1'b0;
		
		read_cnt <= ZERO[ADDR_BITS - 1:0];
		fifo_aclr <= 1'b0;
		read_req_ack <= 1'b0;
	end
	else
		case(state)
			S_IDLE:
			begin
				if(read_req_d2 == 1'b1 && Sdr_init_done)
				begin
					state <= S_ACK;
				end
				read_req_ack <= 1'b0;
			end
			S_ACK:
			begin
				if(read_req_d2 == 1'b0)
				begin
					state <= S_CHECK_FIFO;
					fifo_aclr <= 1'b0;
					read_req_ack <= 1'b0;
				end
				else
				begin
					//read request response
					read_req_ack <= 1'b1;
					//FIFO reset
					fifo_aclr <= 1'b1;
					read_len_latch <= read_len_d1;
				end
				read_cnt <= ZERO[ADDR_BITS - 1:0];
			end
			S_CHECK_FIFO:
			begin
				if(read_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				else if(burst_start)
				begin
					state <= S_READ_BURST;
					App_rd_en_r <= 1'b1;
				end
			end
			
			S_READ_BURST:
			begin
				//burst finish  
				if(rd_burst_finish == 1'b1)
				begin
					App_rd_en_r <= 1'b0;
					state <= S_READ_BURST_END;
					//read counter + burst length
					read_cnt <= read_cnt + burst_len;
					//the next burst read address is generated
				end     
			end
			S_READ_BURST_END:
			begin
				if(read_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				//otherwise the read is complete
				else if(read_cnt < read_len_latch)
				begin
					state <= S_CHECK_FIFO;
				end
				else
				begin
					state <= S_END;
				end
			end
			S_END:
			begin
				state <= S_IDLE;
			end
			default:
				state <= S_IDLE;
		endcase
end
endmodule
