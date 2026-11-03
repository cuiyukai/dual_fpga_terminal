`timescale 1ns/1ps
module frame_read_write
#
(
	parameter MEM_DATA_BITS          = 32,
	parameter READ_DATA_BITS         = 32,
	parameter WRITE_DATA_BITS        = 32,
	parameter ADDR_BITS              = 21,
	parameter BURST_BITS             = 9,//
	parameter BURST_SIZE             = 256,
	parameter WRITE_V_FLIP           = 1,
	parameter FRAME_WIDTH            = 640,
	parameter FRAME_HEIGHT           = 480
) 
(
	input                            rst,                  
	input                            mem_clk,                    // external memory controller user interface clock
	input							 Sdr_init_done,
	input							 Sdr_init_ref_vld,
    input							 Sdr_busy,
    input                            refresh_hold,
    output                           access_idle,
	
	output							 App_rd_en,
	output[ADDR_BITS - 1:0]			 App_rd_addr,
	input							 Sdr_rd_en,					 //read_data_valid
	input[MEM_DATA_BITS - 1:0]		 Sdr_rd_dout,
	
	input                            read_clk,                   // data read module clock
	input                            read_req,
	output                           read_req_ack,               // data read module read request response
	output                           read_finish,                // data read module read request finish
	input[ADDR_BITS - 1:0]           read_addr_0,
	input[ADDR_BITS - 1:0]           read_addr_1,
	input[ADDR_BITS - 1:0]           read_addr_2,
	input[ADDR_BITS - 1:0]           read_addr_3,
	input[1:0]                       read_addr_index,
	input[ADDR_BITS - 1:0]           read_len,                   // data read module read request data length
	input                            read_flip_en,
	input[9:0]                       read_flip_fold,
	input[ADDR_BITS - 1:0]           read_flip_addr,
	input                            read_en,
	output[READ_DATA_BITS  - 1:0]    read_data,                  // read data
	output							 App_wr_en,
	output[ADDR_BITS - 1:0]			 App_wr_addr,
	output[MEM_DATA_BITS - 1:0]	 App_wr_din,
	output[3:0]						 App_wr_dm,
	output                           write_fifo_full,
	
	//
	input                            write_clk,                  // data write module clock
	input                            write_v_flip,
	input                            write_req,
	input                            write_abort_toggle,
	output                           write_req_ack,              // data write module write request response
	output                           write_finish,               // data write module write request finish
	input[ADDR_BITS - 1:0]           write_addr_0,
	input[ADDR_BITS - 1:0]           write_addr_1,
	input[ADDR_BITS - 1:0]           write_addr_2,
	input[ADDR_BITS - 1:0]           write_addr_3,
	input[1:0]                       write_addr_index,
	input[ADDR_BITS - 1:0]           write_len,                  // data write module write request data length
	input                            write_en,                   // data write module write request for one data
	input[WRITE_DATA_BITS - 1:0]     write_data                 // write data

);
wire[BURST_BITS - 1:0]                           wrusedw;                    // write used words
wire[BURST_BITS - 1:0]                           rdusedw;                    // read used words

wire 								 App_rd_busy;
wire								 App_wr_busy;

wire                                 read_fifo_aclr;             // fifo Asynchronous clear
wire                                 write_fifo_aclr;            // fifo Asynchronous clear

wire O_wr_busy;
wire O_rd_busy;

assign App_wr_busy = O_wr_busy | refresh_hold | Sdr_busy | Sdr_init_ref_vld;
assign App_rd_busy = O_rd_busy | refresh_hold | Sdr_busy | Sdr_init_ref_vld;
assign access_idle = !O_wr_busy && !O_rd_busy;
assign App_wr_dm = 4'b0000;

//instantiate an asynchronous FIFO 
wfifo_32_32_512 write_buf
	(
	.clkr                      	(mem_clk                  ),          // Read side clock
	.clkw                      	(write_clk                ),          // Write side clock
	.rst                       	(write_fifo_aclr          ),          // Asynchronous clear
	.we                      	(write_en                 ),          // Write Request
	.re                      	(App_wr_en        		  ),          // Read Request
	.di                       	(write_data               ),          // Input Data
	.empty_flag                 (                         ),          // Read side Empty flag
	.full_flag                  (write_fifo_full           ),          // Write side Full flag
	.wrusedw                	(              	  		  ),          // Read Used Words
	.rdusedw                	(rdusedw                  ),          // Write Used Words
	.dout                       (App_wr_din		          )
);

frame_fifo_write
#
(
	.MEM_DATA_BITS              (MEM_DATA_BITS            ),
	.ADDR_BITS                  (ADDR_BITS                ),
	.BURST_BITS                 (BURST_BITS               ),
	.BURST_SIZE                 (BURST_SIZE               ),
	.WRITE_V_FLIP               (WRITE_V_FLIP             ),
	.FRAME_WIDTH                (FRAME_WIDTH              ),
	.FRAME_HEIGHT               (FRAME_HEIGHT             )
) 
frame_fifo_write_m0              
(  
	.rst                        (rst                      ),
	.mem_clk                    (mem_clk                  ),
    .Sdr_init_done				(Sdr_init_done),
	.Sdr_init_ref_vld			(Sdr_init_ref_vld),
    .Sdr_busy					(Sdr_busy),
	.App_rd_busy				(App_rd_busy),
    .O_wr_busy					(O_wr_busy),
	
	.App_wr_en					(App_wr_en),
    .App_wr_addr				(App_wr_addr),
	.write_req                  (write_req                ),
	.write_v_flip              (write_v_flip             ),
	.write_abort_toggle         (write_abort_toggle       ),
	.write_req_ack              (write_req_ack            ),
	.write_finish               (write_finish             ),
	.write_addr_0               (write_addr_0             ),
	.write_addr_1               (write_addr_1             ),
	.write_addr_2               (write_addr_2             ),
	.write_addr_3               (write_addr_3             ),
	.write_addr_index           (write_addr_index         ),    
	.write_len                  (write_len                ),
	.fifo_aclr                  (write_fifo_aclr          ),
	.rdusedw                 	(rdusedw                  )
);

//instantiate an asynchronous FIFO 
rfifo_32_32_512 read_buf
	(
	.clkr                     	(read_clk                   ),          // Read side clock
	.clkw                     	(mem_clk                    ),          // Write side clock
	.rst                      	(read_fifo_aclr             ),          // Asynchronous clear
	.we                     	(Sdr_rd_en       			),          // Write Request
	.re                     	(read_en                    ),          // Read Request
	.di                      	(Sdr_rd_dout                ),          // Input Data
	.empty_flag					(                           ),          // Read side Empty flag
	.full_flag					(                           ),          // Write side Full flag
	.wrusedw                	(wrusedw         	  	  		 ),          // Read Used Words
	.rdusedw                	(                  			 ),          // Write Used Words
	.dout						(read_data                  )
);

frame_fifo_read
#
(
	.MEM_DATA_BITS              (MEM_DATA_BITS            ),
	.ADDR_BITS                  (ADDR_BITS                ),
	.BURST_BITS                 (BURST_BITS               ),
	.BURST_SIZE                 (BURST_SIZE               ),
	.FRAME_WIDTH                (FRAME_WIDTH              )
)
frame_fifo_read_m0
(
	.rst                        (rst                      ),
	.mem_clk                    (mem_clk                  ),
    .Sdr_init_done				(Sdr_init_done),
	.Sdr_init_ref_vld			(Sdr_init_ref_vld),
    .Sdr_busy					(Sdr_busy),
    .Sdr_rd_en					(Sdr_rd_en),
	.App_wr_busy				(App_wr_busy),
    .O_rd_busy					(O_rd_busy),
	
    .App_rd_en					(App_rd_en),
    .App_rd_addr				(App_rd_addr),
	.read_req                   (read_req                 ),
	.read_req_ack               (read_req_ack             ),
	.read_finish                (read_finish              ),
	.read_addr_0                (read_addr_0              ),
	.read_addr_1                (read_addr_1              ),
	.read_addr_2                (read_addr_2              ),
	.read_addr_3                (read_addr_3              ),
	.read_addr_index            (read_addr_index          ),    
	.read_len                   (read_len                 ),
	.fifo_aclr                  (read_fifo_aclr           ),
	.wrusedw                	(wrusedw                  ),
	.read_flip_en               (read_flip_en             ),
	.read_flip_fold             (read_flip_fold           ),
	.read_flip_addr             (read_flip_addr           )
);

endmodule
