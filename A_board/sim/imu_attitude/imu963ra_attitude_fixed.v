`timescale 1ns / 1ps

module imu963ra_attitude_fixed #(
    parameter integer SAMPLE_HZ = 25
)(
    input                    clk,
    input                    rst_n,
    input                    sample_valid,
    input                    calibrated,
    input signed [15:0]      ax,
    input signed [15:0]      ay,
    input signed [15:0]      az,
    input signed [15:0]      gx,
    input signed [15:0]      gy,
    input signed [15:0]      gz,
    output reg signed [15:0] roll_deg_x10,
    output reg signed [15:0] pitch_deg_x10,
    output reg signed [15:0] yaw_deg_x10,
    output reg               attitude_valid
);

localparam signed [31:0] Q30_ONE       = 32'sd1073741824;
localparam signed [31:0] Q15_ONE       = 32'sd32768;
localparam signed [31:0] DT_Q16        = (65536 + SAMPLE_HZ/2) / SAMPLE_HZ;
localparam signed [31:0] HALF_DT_Q16   = (32768 + SAMPLE_HZ/2) / SAMPLE_HZ;
localparam signed [31:0] KP_STATIC_Q16 = 32'sd26214; // 0.40
localparam signed [31:0] KP_MOTION_Q16 = 32'sd20972; // 0.32
localparam signed [31:0] KP_START_Q16  = 32'sd655360; // 10.0, startup aid
localparam signed [31:0] KI_Q16        = 32'sd983;   // 0.015
localparam signed [31:0] GYRO_RAD_Q16  = 32'sd80;    // 0.01745329/14.3 * 65536

reg signed [31:0] q0, q1, q2, q3;
reg signed [31:0] acc_fx, acc_fy, acc_fz;
reg signed [31:0] acc_err_x, acc_err_y, acc_err_z;
reg               started;
reg [3:0]         startup_count;

reg signed [31:0] raw_ax_q15, raw_ay_q15, raw_az_q15;
reg signed [31:0] filt_x_n, filt_y_n, filt_z_n;
reg signed [31:0] norm_q15;
reg signed [31:0] norm_gain_q15;
reg signed [31:0] abs_x, abs_y, abs_z;
reg signed [31:0] max_abs, mid_abs, min_abs;
reg signed [31:0] norm2_q30;
reg signed [31:0] nax, nay, naz;
reg signed [31:0] grav_x, grav_y, grav_z;
reg signed [31:0] err_x, err_y, err_z;
reg signed [31:0] int_x_n, int_y_n, int_z_n;
reg signed [31:0] omega_x, omega_y, omega_z;
reg signed [31:0] kp_q16;
reg signed [31:0] nq0, nq1, nq2, nq3;
reg signed [31:0] norm_err;
reg signed [31:0] rot00, rot01, rot02, rot12, rot22;
reg signed [31:0] angle_pitch;
reg [1:0] angle_state;
reg [3:0] cordic_iter;
reg signed [34:0] cordic_x, cordic_y;
reg signed [31:0] cordic_angle;
reg signed [15:0] roll_pending;

reg signed [63:0] wide0, wide1, wide2, wide3;
reg signed [63:0] wide4, wide5, wide6, wide7;
reg signed [63:0] wide8;

function signed [31:0] abs32;
    input signed [31:0] value;
    begin
        abs32 = value[31] ? -value : value;
    end
endfunction

function signed [31:0] cordic_step_angle;
    input [3:0] step;
    begin
        case (step)
            4'd0: cordic_step_angle = 32'sd4500;
            4'd1: cordic_step_angle = 32'sd2657;
            4'd2: cordic_step_angle = 32'sd1404;
            4'd3: cordic_step_angle = 32'sd713;
            4'd4: cordic_step_angle = 32'sd358;
            4'd5: cordic_step_angle = 32'sd179;
            4'd6: cordic_step_angle = 32'sd90;
            4'd7: cordic_step_angle = 32'sd45;
            4'd8: cordic_step_angle = 32'sd22;
            4'd9: cordic_step_angle = 32'sd11;
            4'd10: cordic_step_angle = 32'sd6;
            4'd11: cordic_step_angle = 32'sd3;
            4'd12: cordic_step_angle = 32'sd1;
            default: cordic_step_angle = 32'sd1;
        endcase
    end
endfunction

function signed [31:0] asin_deg10;
    input signed [31:0] value;
    reg signed [31:0] mag;
    reg signed [31:0] result;
    reg signed [63:0] product;
    begin
        mag = abs32(value);
        if (mag >= 32'sd1073741824) result = 900;
        else if (mag >= 32'sd1069547520) begin
            product = (mag - 32'sd1069547520) * 32'sd51;
            result = 849 + (product >>> 22);
        end else if (mag >= 32'sd1065353216) begin
            product = (mag - 32'sd1065353216) * 32'sd20;
            result = 829 + (product >>> 22);
        end else if (mag >= 32'sd1056964608) begin
            product = (mag - 32'sd1056964608) * 32'sd30;
            result = 799 + (product >>> 23);
        end else if (mag >= 32'sd1040187392) begin
            product = (mag - 32'sd1040187392) * 32'sd43;
            result = 756 + (product >>> 24);
        end else if (mag >= 32'sd1006632960) begin
            product = (mag - 32'sd1006632960) * 32'sd60;
            result = 696 + (product >>> 25);
        end else if (mag >= 32'sd939524096) begin
            product = (mag - 32'sd939524096) * 32'sd86;
            result = 610 + (product >>> 26);
        end else if (mag >= 32'sd805306368) begin
            product = (mag - 32'sd805306368) * 32'sd124;
            result = 486 + (product >>> 27);
        end else if (mag >= 32'sd671088640) begin
            product = (mag - 32'sd671088640) * 32'sd99;
            result = 387 + (product >>> 27);
        end else if (mag >= 32'sd536870912) begin
            product = (mag - 32'sd536870912) * 32'sd87;
            result = 300 + (product >>> 27);
        end else if (mag >= 32'sd402653184) begin
            product = (mag - 32'sd402653184) * 32'sd80;
            result = 220 + (product >>> 27);
        end else if (mag >= 32'sd268435456) begin
            product = (mag - 32'sd268435456) * 32'sd75;
            result = 145 + (product >>> 27);
        end else if (mag >= 32'sd134217728) begin
            product = (mag - 32'sd134217728) * 32'sd73;
            result = 72 + (product >>> 27);
        end else begin
            product = mag * 32'sd72;
            result = product >>> 27;
        end
        asin_deg10 = value[31] ? -result : result;
    end
endfunction

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        q0 <= Q30_ONE;
        q1 <= 0;
        q2 <= 0;
        q3 <= 0;
        acc_fx <= 0;
        acc_fy <= 0;
        acc_fz <= Q15_ONE;
        acc_err_x <= 0;
        acc_err_y <= 0;
        acc_err_z <= 0;
        started <= 1'b0;
        startup_count <= 0;
        roll_deg_x10 <= 0;
        pitch_deg_x10 <= 0;
        yaw_deg_x10 <= 0;
        attitude_valid <= 1'b0;
        angle_state <= 2'd0;
        cordic_iter <= 4'd0;
        cordic_x <= 35'sd0;
        cordic_y <= 35'sd0;
        cordic_angle <= 32'sd0;
        roll_pending <= 16'sd0;
    end else begin
        attitude_valid <= 1'b0;
        if (angle_state != 2'd0) begin
            if (cordic_iter < 4'd14) begin
                if (cordic_y >= 0) begin
                    cordic_x <= cordic_x + (cordic_y >>> cordic_iter);
                    cordic_y <= cordic_y - (cordic_x >>> cordic_iter);
                    cordic_angle <= cordic_angle + cordic_step_angle(cordic_iter);
                end else begin
                    cordic_x <= cordic_x - (cordic_y >>> cordic_iter);
                    cordic_y <= cordic_y + (cordic_x >>> cordic_iter);
                    cordic_angle <= cordic_angle - cordic_step_angle(cordic_iter);
                end
                cordic_iter <= cordic_iter + 1'b1;
            end else if (angle_state == 2'd1) begin
                roll_pending <= cordic_angle / 10;
                angle_state <= 2'd2;
                cordic_iter <= 4'd0;
                if (rot00 < 0) begin
                    cordic_x <= -{{3{rot00[31]}}, rot00};
                    cordic_y <= -{{3{rot01[31]}}, rot01};
                    cordic_angle <= (rot01 >= 0) ? 32'sd18000 : -32'sd18000;
                end else begin
                    cordic_x <= {{3{rot00[31]}}, rot00};
                    cordic_y <= {{3{rot01[31]}}, rot01};
                    cordic_angle <= 32'sd0;
                end
            end else begin
                roll_deg_x10 <= roll_pending;
                pitch_deg_x10 <= angle_pitch[15:0];
                yaw_deg_x10 <= cordic_angle / 10;
                attitude_valid <= 1'b1;
                angle_state <= 2'd0;
            end
        end else if (sample_valid && calibrated) begin
            raw_ax_q15 = {{16{ax[15]}}, ax} <<< 3;
            raw_ay_q15 = {{16{ay[15]}}, ay} <<< 3;
            raw_az_q15 = {{16{az[15]}}, az} <<< 3;

            wide0 = raw_ax_q15 * raw_ax_q15;
            wide1 = raw_ay_q15 * raw_ay_q15;
            wide2 = raw_az_q15 * raw_az_q15;
            wide8 = wide0 + wide1 + wide2;
            if ((wide8 >= 64'sd869730878) && (wide8 <= 64'sd1300647445)) begin
                filt_x_n = (acc_fx * 4 + raw_ax_q15) / 5;
                filt_y_n = (acc_fy * 4 + raw_ay_q15) / 5;
                filt_z_n = (acc_fz * 4 + raw_az_q15) / 5;
                kp_q16 = KP_STATIC_Q16;
            end else begin
                filt_x_n = (acc_fx + raw_ax_q15) >>> 1;
                filt_y_n = (acc_fy + raw_ay_q15) >>> 1;
                filt_z_n = (acc_fz + raw_az_q15) >>> 1;
                kp_q16 = KP_MOTION_Q16;
            end
            if (!started) begin
                filt_x_n = raw_ax_q15;
                filt_y_n = raw_ay_q15;
                filt_z_n = raw_az_q15;
                started <= 1'b1;
            end
            if (startup_count < 4'd3) begin
                startup_count <= startup_count + 1'b1;
                kp_q16 = KP_START_Q16;
            end
            acc_fx <= filt_x_n;
            acc_fy <= filt_y_n;
            acc_fz <= filt_z_n;

            abs_x = abs32(filt_x_n);
            abs_y = abs32(filt_y_n);
            abs_z = abs32(filt_z_n);
            max_abs = abs_x;
            if (abs_y > max_abs) max_abs = abs_y;
            if (abs_z > max_abs) max_abs = abs_z;
            min_abs = abs_x;
            if (abs_y < min_abs) min_abs = abs_y;
            if (abs_z < min_abs) min_abs = abs_z;
            mid_abs = abs_x + abs_y + abs_z - max_abs - min_abs;
            norm_q15 = max_abs + (mid_abs >>> 1) + (min_abs >>> 2);
            if (norm_q15 < 32'sd1024) begin
                nax = 0;
                nay = 0;
                naz = Q15_ONE;
            end else if (norm_q15 >= 32'sd65536) begin
                nax = 0;
                nay = 0;
                naz = 0;
            end else begin
                // One Newton reciprocal step around 1 g, valid for the
                // expected 0.9..1.1 g static range.
                norm_gain_q15 = 32'sd65536 - norm_q15;
                wide0 = filt_x_n * norm_gain_q15;
                wide1 = filt_y_n * norm_gain_q15;
                wide2 = filt_z_n * norm_gain_q15;
                nax = wide0 >>> 15;
                nay = wide1 >>> 15;
                naz = wide2 >>> 15;
            end

            wide0 = q1 * q3;
            wide1 = q0 * q2;
            grav_x = ((wide0 - wide1) >>> 44);
            wide2 = q0 * q1;
            wide3 = q2 * q3;
            grav_y = ((wide2 + wide3) >>> 44);
            wide4 = q0 * q0;
            wide5 = q1 * q1;
            wide6 = q2 * q2;
            wide7 = q3 * q3;
            grav_z = (wide4 - wide5 - wide6 + wide7) >>> 45;

            err_x = ((nay * grav_z) - (naz * grav_y)) >>> 15;
            err_y = ((naz * grav_x) - (nax * grav_z)) >>> 15;
            err_z = ((nax * grav_y) - (nay * grav_x)) >>> 15;

            int_x_n = acc_err_x + (((err_x * DT_Q16) >>> 16));
            int_y_n = acc_err_y + (((err_y * DT_Q16) >>> 16));
            int_z_n = acc_err_z + (((err_z * DT_Q16) >>> 16));
            if (int_x_n > 32'sd32768) int_x_n = 32'sd32768;
            if (int_x_n < -32'sd32768) int_x_n = -32'sd32768;
            if (int_y_n > 32'sd32768) int_y_n = 32'sd32768;
            if (int_y_n < -32'sd32768) int_y_n = -32'sd32768;
            if (int_z_n > 32'sd32768) int_z_n = 32'sd32768;
            if (int_z_n < -32'sd32768) int_z_n = -32'sd32768;
            acc_err_x <= int_x_n;
            acc_err_y <= int_y_n;
            acc_err_z <= int_z_n;

            omega_x = ($signed(gx) * GYRO_RAD_Q16) + ((err_x * kp_q16) >>> 15) + ((int_x_n * KI_Q16) >>> 15);
            omega_y = ($signed(gy) * GYRO_RAD_Q16) + ((err_y * kp_q16) >>> 15) + ((int_y_n * KI_Q16) >>> 15);
            omega_z = ($signed(gz) * GYRO_RAD_Q16) + ((err_z * kp_q16) >>> 15) + ((int_z_n * KI_Q16) >>> 15);

            // Quaternion differential equation, dt = 1/25 s.
            wide0 = (-q1 * omega_x) - (q2 * omega_y) - (q3 * omega_z);
            wide1 = ( q0 * omega_x) + (q2 * omega_z) - (q3 * omega_y);
            wide2 = ( q0 * omega_y) - (q1 * omega_z) + (q3 * omega_x);
            wide3 = ( q0 * omega_z) + (q1 * omega_y) - (q2 * omega_x);
            nq0 = q0 + ((wide0 >>> 16) * HALF_DT_Q16 >>> 16);
            nq1 = q1 + ((wide1 >>> 16) * HALF_DT_Q16 >>> 16);
            nq2 = q2 + ((wide2 >>> 16) * HALF_DT_Q16 >>> 16);
            nq3 = q3 + ((wide3 >>> 16) * HALF_DT_Q16 >>> 16);

            // First-order quaternion normalization: q *= 1 + (1-|q|^2)/2.
            wide4 = nq0 * nq0;
            wide5 = nq1 * nq1;
            wide6 = nq2 * nq2;
            wide7 = nq3 * nq3;
            norm2_q30 = (wide4 + wide5 + wide6 + wide7) >>> 30;
            norm_err = Q30_ONE - norm2_q30;
            nq0 = nq0 + ((nq0 * norm_err) >>> 31);
            nq1 = nq1 + ((nq1 * norm_err) >>> 31);
            nq2 = nq2 + ((nq2 * norm_err) >>> 31);
            nq3 = nq3 + ((nq3 * norm_err) >>> 31);
            q0 <= nq0;
            q1 <= nq1;
            q2 <= nq2;
            q3 <= nq3;

            wide0 = nq0 * nq0;
            wide1 = nq1 * nq1;
            wide2 = nq2 * nq2;
            wide3 = nq3 * nq3;
            rot00 = (wide0 + wide1 - wide2 - wide3) >>> 30;
            wide4 = nq1 * nq2;
            wide5 = nq0 * nq3;
            rot01 = (2 * (wide4 + wide5)) >>> 30;
            wide4 = nq1 * nq3;
            wide5 = nq0 * nq2;
            rot02 = (2 * (wide4 - wide5)) >>> 30;
            wide4 = nq2 * nq3;
            wide5 = nq0 * nq1;
            rot12 = (2 * (wide4 + wide5)) >>> 30;
            rot22 = (wide0 - wide1 - wide2 + wide3) >>> 30;
            angle_pitch = -asin_deg10(rot02);
            if (angle_pitch > 900) angle_pitch = 900;
            if (angle_pitch < -900) angle_pitch = -900;
            angle_state <= 2'd1;
            cordic_iter <= 4'd0;
            if (rot22 < 0) begin
                cordic_x <= -{{3{rot22[31]}}, rot22};
                cordic_y <= -{{3{rot12[31]}}, rot12};
                cordic_angle <= (rot12 >= 0) ? 32'sd18000 : -32'sd18000;
            end else begin
                cordic_x <= {{3{rot22[31]}}, rot22};
                cordic_y <= {{3{rot12[31]}}, rot12};
                cordic_angle <= 32'sd0;
            end
        end
    end
end

endmodule
