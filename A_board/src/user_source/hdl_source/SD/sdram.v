`timescale 1ns / 1ps

`define DEBUG

`include "../user_source/hdl_source/include/global_def.v"

module sdram(
    `ifdef SIMULATION
	output					SDRAM_CLK,
	output					SDR_CKE,
	output					SDR_RAS,
	output					SDR_CAS,
	output					SDR_WE,
	output	[`BA_WIDTH-1:0]	SDR_BA,
	output	[`ROW_WIDTH-1:0]SDR_ADDR,
	output	[`DM_WIDTH-1:0]	SDR_DM,
	inout	[`DATA_WIDTH-1:0]	SDR_DQ,	
	`endif
    input   					Clk,
    input						Clk_sft,
    input						Rst,
    
    output 						Sdr_init_done,
    output						Sdr_init_ref_vld,
    output						Sdr_busy,
    input                       App_access_idle,
    output                      App_refresh_hold,
    
    input						App_wr_en,
	input [`ADDR_WIDTH-1:0]		App_wr_addr,  	
	input [`DM_WIDTH-1:0]		App_wr_dm,
	input [`DATA_WIDTH-1:0]		App_wr_din,	//synthesis keep
    
    input 						App_rd_en,
	input [`ADDR_WIDTH-1:0]		App_rd_addr,
	output 						Sdr_rd_en,
	output [`DATA_WIDTH-1:0]	Sdr_rd_dout	//synthesis keep
    
);
`ifndef SIMULATION
wire  			SDRAM_CLK;
wire 			SDR_RAS;
wire  			SDR_CAS;
wire  			SDR_WE;
wire  [`BA_WIDTH-1:0]	SDR_BA; 
wire  [`ROW_WIDTH-1:0]	SDR_ADDR;
wire  [`DATA_WIDTH-1:0]	SDR_DQ ;
wire  [`DM_WIDTH-1:0]	SDR_DM; 
`endif

localparam integer REFRESH_IDLE_CYCLES = (`SELF_REFRESH_INTERVAL / 2);
localparam [2:0] REF_IDLE=0, REF_DRAIN=1, REF_REQUEST=2,
                 REF_ACTIVE=3, REF_RECOVER=4;
reg [2:0] refresh_state;
reg [15:0] app_ref_cnt;
reg [4:0] refresh_guard;
reg app_access_idle_r;
wire app_ref_req;
assign App_refresh_hold = !Sdr_init_done || (refresh_state != REF_IDLE);
assign app_ref_req = (refresh_state == REF_REQUEST) && !Sdr_init_ref_vld;

always @(posedge Clk or posedge Rst) begin
    if (Rst) begin
        refresh_state <= REF_IDLE;
        app_ref_cnt <= 16'd0;
        refresh_guard <= 5'd0;
        app_access_idle_r <= 1'b0;
    end else if (!Sdr_init_done) begin
        app_access_idle_r <= 1'b0;
        refresh_state <= REF_IDLE;
        app_ref_cnt <= 16'd0;
        refresh_guard <= 5'd0;
    end else begin
        app_access_idle_r <= App_access_idle;
        case (refresh_state)
            REF_IDLE: begin
                if (app_ref_cnt >= REFRESH_IDLE_CYCLES - 1) begin
                    refresh_state <= REF_DRAIN;
                    refresh_guard <= 5'd0;
                end else
                    app_ref_cnt <= app_ref_cnt + 16'd1;
            end
            REF_DRAIN: begin
                // Include controller busy and returned data, not only enables.
                if (!app_access_idle_r || App_rd_en || App_wr_en ||
                    Sdr_rd_en || Sdr_busy || Sdr_init_ref_vld)
                    refresh_guard <= 5'd0;
                else if (refresh_guard == 5'd15) begin
                    refresh_state <= REF_REQUEST;
                    refresh_guard <= 5'd0;
                end else
                    refresh_guard <= refresh_guard + 5'd1;
            end
            REF_REQUEST: begin
                // Hold until refresh STARTS; keep traffic blocked until END.
                if (Sdr_init_ref_vld)
                    refresh_state <= REF_ACTIVE;
            end
            REF_ACTIVE: begin
                if (!Sdr_init_ref_vld && !Sdr_busy) begin
                    refresh_state <= REF_RECOVER;
                    refresh_guard <= 5'd0;
                end
            end
            REF_RECOVER: begin
                if (Sdr_init_ref_vld || Sdr_busy)
                    refresh_guard <= 5'd0;
                else if (refresh_guard == 5'd15) begin
                    refresh_state <= REF_IDLE;
                    app_ref_cnt <= 16'd0;
                end else
                    refresh_guard <= refresh_guard + 5'd1;
            end
            default: begin
                refresh_state <= REF_DRAIN;
                refresh_guard <= 5'd0;
                app_ref_cnt <= 16'd0;
            end
        endcase
    end
end

sdr_as_ram  #( .self_refresh_open(1'b0))
	u2_ram( 
		.Sdr_clk(Clk),
		.Sdr_clk_sft(Clk_sft),
		.Rst(Rst),
			  			  
		.Sdr_init_done(Sdr_init_done),
		.Sdr_init_ref_vld(Sdr_init_ref_vld),
		.Sdr_busy(Sdr_busy),
		
		.App_ref_req(app_ref_req),
		
        .App_wr_en(App_wr_en), 
        .App_wr_addr(App_wr_addr),  	
		.App_wr_dm(App_wr_dm),
		.App_wr_din(App_wr_din),

		.App_rd_en(App_rd_en),
		.App_rd_addr(App_rd_addr),
		.Sdr_rd_en	(Sdr_rd_en),
		.Sdr_rd_dout(Sdr_rd_dout),
	
		.SDRAM_CLK(SDRAM_CLK),
		.SDR_RAS(SDR_RAS),
		.SDR_CAS(SDR_CAS),
		.SDR_WE(SDR_WE),
		.SDR_BA(SDR_BA),
		.SDR_ADDR(SDR_ADDR),
		.SDR_DM(SDR_DM),
		.SDR_DQ(SDR_DQ)	
	);

assign SDR_CKE=1'b1;

//`ifndef SIMULATION
	EG_PHY_SDRAM_2M_32 sdram(
		.clk(SDRAM_CLK),
		.ras_n(SDR_RAS),
		.cas_n(SDR_CAS),
		.we_n(SDR_WE),
		.addr(SDR_ADDR[10:0]),
		.ba(SDR_BA),
		.dq(SDR_DQ),
		.cs_n(1'b0),
		.dm0(SDR_DM[0]),
		.dm1(SDR_DM[1]),
		.dm2(SDR_DM[2]),
		.dm3(SDR_DM[3]),
		.cke(1'b1)
		);
//`endif

endmodule 
