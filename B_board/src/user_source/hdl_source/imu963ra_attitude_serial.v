`timescale 1ns / 1ps

module imu963ra_attitude_serial (
    input clk,
    input rst_n,
    input sample_valid,
    input calibrated,
    input signed [15:0] ax,
    input signed [15:0] ay,
    input signed [15:0] az,
    input signed [15:0] gx,
    input signed [15:0] gy,
    input signed [15:0] gz,
    output reg signed [15:0] roll_deg_x10,
    output reg signed [15:0] pitch_deg_x10,
    output reg signed [15:0] yaw_deg_x10,
    output reg attitude_valid,
    output wire signed [17:0] quat_w,
    output wire signed [17:0] quat_x,
    output wire signed [17:0] quat_y,
    output wire signed [17:0] quat_z
);

localparam signed [17:0] ONE = 18'sd16384;
localparam [2:0] IDLE = 3'd0, NORM = 3'd1, NORM_SUM = 3'd2,
                 NORM_GAIN = 3'd3, FILTER = 3'd4, MULTIPLY = 3'd5,
                 ROLL_ANGLE = 3'd6, YAW_ANGLE = 3'd7;

reg [2:0] state;
reg [5:0] step;
reg [1:0] phase;
reg [3:0] cordic_iter;
reg [3:0] startup_count;
reg first_sample;
reg static_state;
reg signed [17:0] q0, q1, q2, q3;
assign quat_w = q0;
assign quat_x = q1;
assign quat_y = q2;
assign quat_z = q3;
reg signed [17:0] n0, n1, n2, n3;
reg signed [17:0] fax, fay, faz;
reg signed [17:0] nax, nay, naz;
reg signed [17:0] norm_gain;
reg signed [17:0] grav_x, grav_y, grav_z;
reg signed [17:0] err_x, err_y, err_z;
reg signed [17:0] int_x, int_y, int_z;
reg signed [17:0] kp;
reg signed [17:0] omega_x, omega_y, omega_z;
reg signed [17:0] mul_a, mul_b;
wire signed [35:0] product = mul_a * mul_b;
reg signed [35:0] temp_product;
reg signed [39:0] d0, d1, d2, d3;
reg signed [35:0] sq0, sq1, sq2, sq3;
reg signed [17:0] rot00, rot01, rot02, rot12, rot22;
reg signed [19:0] cordic_x, cordic_y;
reg signed [15:0] cordic_angle;
reg signed [15:0] roll_pending;
reg signed [17:0] raw_x, raw_y, raw_z;
reg signed [17:0] filtered_x, filtered_y, filtered_z;
reg signed [17:0] abs_x, abs_y, abs_z, max_abs, min_abs, mid_abs;
reg signed [17:0] approx_norm;
reg signed [39:0] delta_product;
reg signed [17:0] int_next;
reg signed [17:0] gyro_correction;
reg signed [17:0] gyro_corr_pending;
reg signed [39:0] omega_base_pending;
reg signed [17:0] pitch_magnitude;
reg signed [17:0] pitch_delta;
reg signed [17:0] pitch_slope;
reg signed [15:0] pitch_base;
reg [3:0] pitch_shift;
reg pitch_negative;
reg signed [35:0] pitch_product;

function signed [17:0] abs18;
    input signed [17:0] value;
    begin abs18 = value < 0 ? -value : value; end
endfunction

function signed [17:0] limit18;
    input signed [39:0] value;
    begin
        if (value > 40'sd131071) limit18 = 18'sd131071;
        else if (value < -40'sd131072) limit18 = -18'sd131072;
        else limit18 = value[17:0];
    end
endfunction

function signed [15:0] cordic_step_angle;
    input [3:0] index;
    begin
        case (index)
            0: cordic_step_angle = 450;
            1: cordic_step_angle = 266;
            2: cordic_step_angle = 140;
            3: cordic_step_angle = 71;
            4: cordic_step_angle = 36;
            5: cordic_step_angle = 18;
            6: cordic_step_angle = 9;
            7: cordic_step_angle = 4;
            8: cordic_step_angle = 2;
            9: cordic_step_angle = 1;
            10: cordic_step_angle = 1;
            default: cordic_step_angle = 0;
        endcase
    end
endfunction

function signed [15:0] asin_deg_x10;
    input signed [17:0] value;
    reg signed [17:0] magnitude;
    reg signed [31:0] result;
    begin
        magnitude = abs18(value);
        if (magnitude >= 16384) result = 900;
        else if (magnitude >= 16320) result = 849 + ((magnitude - 16320) * 51 >>> 6);
        else if (magnitude >= 16256) result = 829 + ((magnitude - 16256) * 20 >>> 6);
        else if (magnitude >= 16128) result = 799 + ((magnitude - 16128) * 30 >>> 7);
        else if (magnitude >= 15872) result = 756 + ((magnitude - 15872) * 43 >>> 8);
        else if (magnitude >= 15360) result = 696 + ((magnitude - 15360) * 60 >>> 9);
        else if (magnitude >= 14336) result = 610 + ((magnitude - 14336) * 86 >>> 10);
        else if (magnitude >= 12288) result = 486 + ((magnitude - 12288) * 124 >>> 11);
        else if (magnitude >= 10240) result = 387 + ((magnitude - 10240) * 99 >>> 11);
        else if (magnitude >= 8192) result = 300 + ((magnitude - 8192) * 87 >>> 11);
        else if (magnitude >= 6144) result = 220 + ((magnitude - 6144) * 80 >>> 11);
        else if (magnitude >= 4096) result = 145 + ((magnitude - 4096) * 75 >>> 11);
        else if (magnitude >= 2048) result = 72 + ((magnitude - 2048) * 73 >>> 11);
        else result = magnitude * 72 >>> 11;
        asin_deg_x10 = value < 0 ? -result : result;
    end
endfunction

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        step <= 0;
        phase <= 0;
        cordic_iter <= 0;
        startup_count <= 0;
        first_sample <= 1'b1;
        static_state <= 1'b1;
        q0 <= ONE; q1 <= 0; q2 <= 0; q3 <= 0;
        n0 <= ONE; n1 <= 0; n2 <= 0; n3 <= 0;
        fax <= 0; fay <= 0; faz <= ONE;
        nax <= 0; nay <= 0; naz <= ONE;
        norm_gain <= ONE;
        grav_x <= 0; grav_y <= 0; grav_z <= ONE;
        err_x <= 0; err_y <= 0; err_z <= 0;
        int_x <= 0; int_y <= 0; int_z <= 0;
        kp <= 18'sd6554;
        omega_x <= 0; omega_y <= 0; omega_z <= 0;
        mul_a <= 0; mul_b <= 0;
        temp_product <= 0;
        d0 <= 0; d1 <= 0; d2 <= 0; d3 <= 0;
        sq0 <= 0; sq1 <= 0; sq2 <= 0; sq3 <= 0;
        rot00 <= ONE; rot01 <= 0; rot02 <= 0;
        rot12 <= 0; rot22 <= ONE;
        cordic_x <= 0; cordic_y <= 0; cordic_angle <= 0;
        roll_pending <= 0;
        pitch_magnitude <= 0;
        pitch_delta <= 0;
        pitch_slope <= 0;
        pitch_base <= 0;
        pitch_shift <= 0;
        pitch_negative <= 0;
        pitch_product <= 0;
        gyro_corr_pending <= 0;
        omega_base_pending <= 0;
        roll_deg_x10 <= 0; pitch_deg_x10 <= 0; yaw_deg_x10 <= 0;
        attitude_valid <= 0;
    end else begin
        attitude_valid <= 0;
        case (state)
            IDLE: if (sample_valid && calibrated) begin
                raw_x <= $signed(ax) <<< 2;
                raw_y <= $signed(ay) <<< 2;
                raw_z <= $signed(az) <<< 2;
                state <= NORM;
            end
            NORM: begin
                abs_x <= abs18(raw_x);
                abs_y <= abs18(raw_y);
                abs_z <= abs18(raw_z);
                state <= NORM_SUM;
            end
            NORM_SUM: begin
                max_abs = abs_x;
                if (abs_y > max_abs) max_abs = abs_y;
                if (abs_z > max_abs) max_abs = abs_z;
                min_abs = abs_x;
                if (abs_y < min_abs) min_abs = abs_y;
                if (abs_z < min_abs) min_abs = abs_z;
                mid_abs = abs_x + abs_y + abs_z - max_abs - min_abs;
                approx_norm <= max_abs + (mid_abs >>> 1) + (min_abs >>> 2);
                state <= NORM_GAIN;
            end
            NORM_GAIN: begin
                static_state <= (approx_norm >= 18'sd14746 && approx_norm <= 18'sd18022);
                if (approx_norm < 18'sd8192 || approx_norm >= 18'sd32768)
                    norm_gain <= 0;
                else
                    norm_gain <= 18'sd32768 - approx_norm;
                kp <= (approx_norm >= 18'sd14746 && approx_norm <= 18'sd18022) ?
                      18'sd6554 : 18'sd5243;
                state <= FILTER;
            end
            FILTER: begin
                if (first_sample) begin
                    filtered_x = raw_x;
                    filtered_y = raw_y;
                    filtered_z = raw_z;
                    first_sample <= 1'b0;
                end else if (static_state) begin
                    delta_product = $signed(fax) * 40'sd13 + $signed(raw_x) * 40'sd3;
                    filtered_x = delta_product >>> 4;
                    delta_product = $signed(fay) * 40'sd13 + $signed(raw_y) * 40'sd3;
                    filtered_y = delta_product >>> 4;
                    delta_product = $signed(faz) * 40'sd13 + $signed(raw_z) * 40'sd3;
                    filtered_z = delta_product >>> 4;
                end else begin
                    filtered_x = (fax + raw_x) >>> 1;
                    filtered_y = (fay + raw_y) >>> 1;
                    filtered_z = (faz + raw_z) >>> 1;
                end
                fax <= filtered_x;
                fay <= filtered_y;
                faz <= filtered_z;
                if (startup_count < 3) startup_count <= startup_count + 1'b1;
                state <= MULTIPLY;
                phase <= 0;
                step <= 0;
            end
            MULTIPLY: if (phase == 2'd0) begin
                case (step)
                    0: begin mul_a <= fax; mul_b <= norm_gain; end
                    1: begin mul_a <= fay; mul_b <= norm_gain; end
                    2: begin mul_a <= faz; mul_b <= norm_gain; end
                    3: begin mul_a <= q1; mul_b <= q3; end
                    4: begin mul_a <= q0; mul_b <= q2; end
                    5: begin mul_a <= q0; mul_b <= q1; end
                    6: begin mul_a <= q2; mul_b <= q3; end
                    7: begin mul_a <= q1; mul_b <= q1; end
                    8: begin mul_a <= q2; mul_b <= q2; end
                    9: begin mul_a <= nay; mul_b <= grav_z; end
                    10: begin mul_a <= naz; mul_b <= grav_y; end
                    11: begin mul_a <= naz; mul_b <= grav_x; end
                    12: begin mul_a <= nax; mul_b <= grav_z; end
                    13: begin mul_a <= nax; mul_b <= grav_y; end
                    14: begin mul_a <= nay; mul_b <= grav_x; end
                    15: begin mul_a <= err_x; mul_b <= kp; end
                    16: begin mul_a <= err_y; mul_b <= kp; end
                    17: begin mul_a <= err_z; mul_b <= kp; end
                    18: begin mul_a <= q1; mul_b <= omega_x; end
                    19: begin mul_a <= q2; mul_b <= omega_y; end
                    20: begin mul_a <= q3; mul_b <= omega_z; end
                    21: begin mul_a <= q0; mul_b <= omega_x; end
                    22: begin mul_a <= q2; mul_b <= omega_z; end
                    23: begin mul_a <= q3; mul_b <= omega_y; end
                    24: begin mul_a <= q0; mul_b <= omega_y; end
                    25: begin mul_a <= q1; mul_b <= omega_z; end
                    26: begin mul_a <= q3; mul_b <= omega_x; end
                    27: begin mul_a <= q0; mul_b <= omega_z; end
                    28: begin mul_a <= q1; mul_b <= omega_y; end
                    29: begin mul_a <= q2; mul_b <= omega_x; end
                    30: begin mul_a <= n0; mul_b <= n0; end
                    31: begin mul_a <= n1; mul_b <= n1; end
                    32: begin mul_a <= n2; mul_b <= n2; end
                    33: begin mul_a <= n3; mul_b <= n3; end
                    34: begin mul_a <= n0; mul_b <= norm_gain; end
                    35: begin mul_a <= n1; mul_b <= norm_gain; end
                    36: begin mul_a <= n2; mul_b <= norm_gain; end
                    37: begin mul_a <= n3; mul_b <= norm_gain; end
                    38: begin mul_a <= n1; mul_b <= n2; end
                    39: begin mul_a <= n0; mul_b <= n3; end
                    40: begin mul_a <= n1; mul_b <= n3; end
                    41: begin mul_a <= n0; mul_b <= n2; end
                    42: begin mul_a <= n2; mul_b <= n3; end
                    43: begin mul_a <= n0; mul_b <= n1; end
                    default: begin mul_a <= 0; mul_b <= 0; end
                endcase
                phase <= 2'd1;
            end else if (phase == 2'd1) begin
                phase <= 0;
                step <= step + 1'b1;
                case (step)
                    0: nax <= product >>> 14;
                    1: nay <= product >>> 14;
                    2: naz <= product >>> 14;
                    3: temp_product <= product;
                    4: grav_x <= (2 * (temp_product - product)) >>> 14;
                    5: temp_product <= product;
                    6: grav_y <= (2 * (temp_product + product)) >>> 14;
                    7: sq1 <= product;
                    8: begin
                        sq2 <= product;
                        grav_z <= ONE - (2 * (sq1 + product) >>> 14);
                    end
                    9: temp_product <= product;
                    10: err_x <= (temp_product - product) >>> 14;
                    11: temp_product <= product;
                    12: err_y <= (temp_product - product) >>> 14;
                    13: temp_product <= product;
                    14: err_z <= (temp_product - product) >>> 14;
                    15: begin
                        delta_product = $signed(err_x) * 40'sd655;
                        int_next = int_x + (delta_product >>> (static_state ? 14 : 17));
                        if (int_next > ONE) int_next = ONE;
                        if (int_next < -ONE) int_next = -ONE;
                        int_x <= int_next;
                        delta_product = $signed(err_x) * 40'sd10;
                        gyro_correction = (startup_count < 3) ? (delta_product >>> 2) : (product >>> 16);
                        gyro_corr_pending <= gyro_correction;
                        omega_base_pending <= $signed(gx) * 40'sd5 + (($signed(int_x) * 40'sd246) >>> 16);
                        phase <= 2'd2;
                        step <= step;
                    end
                    16: begin
                        delta_product = $signed(err_y) * 40'sd655;
                        int_next = int_y + (delta_product >>> (static_state ? 14 : 17));
                        if (int_next > ONE) int_next = ONE;
                        if (int_next < -ONE) int_next = -ONE;
                        int_y <= int_next;
                        delta_product = $signed(err_y) * 40'sd10;
                        gyro_correction = (startup_count < 3) ? (delta_product >>> 2) : (product >>> 16);
                        gyro_corr_pending <= gyro_correction;
                        omega_base_pending <= $signed(gy) * 40'sd5 + (($signed(int_y) * 40'sd246) >>> 16);
                        phase <= 2'd2;
                        step <= step;
                    end
                    17: begin
                        delta_product = $signed(err_z) * 40'sd655;
                        int_next = int_z + (delta_product >>> (static_state ? 14 : 17));
                        if (int_next > ONE) int_next = ONE;
                        if (int_next < -ONE) int_next = -ONE;
                        int_z <= int_next;
                        delta_product = $signed(err_z) * 40'sd10;
                        gyro_correction = (startup_count < 3) ? (delta_product >>> 2) : (product >>> 16);
                        gyro_corr_pending <= gyro_correction;
                        omega_base_pending <= $signed(gz) * 40'sd5 + (($signed(int_z) * 40'sd246) >>> 16);
                        phase <= 2'd2;
                        step <= step;
                    end
                    18: d0 <= -product;
                    19: d0 <= d0 - product;
                    20: d0 <= d0 - product;
                    21: d1 <= product;
                    22: d1 <= d1 + product;
                    23: d1 <= d1 - product;
                    24: d2 <= product;
                    25: d2 <= d2 - product;
                    26: d2 <= d2 + product;
                    27: d3 <= product;
                    28: d3 <= d3 + product;
                    29: begin
                        delta_product = d0 * 40'sd328;
                        n0 <= q0 + (delta_product >>> 26);
                        delta_product = d1 * 40'sd328;
                        n1 <= q1 + (delta_product >>> 26);
                        delta_product = d2 * 40'sd328;
                        n2 <= q2 + (delta_product >>> 26);
                        delta_product = (d3 - product) * 40'sd328;
                        n3 <= q3 + (delta_product >>> 26);
                    end
                    30: sq0 <= product;
                    31: sq1 <= product;
                    32: sq2 <= product;
                    33: begin
                        sq3 <= product;
                        norm_gain <= ONE + ((ONE - ((sq0 + sq1 + sq2 + product) >>> 14)) >>> 1);
                        rot00 <= (sq0 + sq1 - sq2 - product) >>> 14;
                        rot22 <= (sq0 - sq1 - sq2 + product) >>> 14;
                    end
                    34: n0 <= product >>> 14;
                    35: n1 <= product >>> 14;
                    36: n2 <= product >>> 14;
                    37: n3 <= product >>> 14;
                    38: temp_product <= product;
                    39: rot01 <= (2 * (temp_product + product)) >>> 14;
                    40: temp_product <= product;
                    41: rot02 <= (2 * (temp_product - product)) >>> 14;
                    42: temp_product <= product;
                    43: begin
                        rot12 <= (2 * (temp_product + product)) >>> 14;
                        pitch_magnitude = abs18(rot02);
                        pitch_negative <= !rot02[17];
                        if (pitch_magnitude >= 18'sd16384) begin
                            pitch_base <= 16'sd900; pitch_delta <= 0; pitch_slope <= 0; pitch_shift <= 0;
                        end else if (pitch_magnitude >= 18'sd16320) begin
                            pitch_base <= 16'sd849; pitch_delta <= pitch_magnitude - 18'sd16320; pitch_slope <= 18'sd51; pitch_shift <= 4'd6;
                        end else if (pitch_magnitude >= 18'sd16256) begin
                            pitch_base <= 16'sd829; pitch_delta <= pitch_magnitude - 18'sd16256; pitch_slope <= 18'sd20; pitch_shift <= 4'd6;
                        end else if (pitch_magnitude >= 18'sd16128) begin
                            pitch_base <= 16'sd799; pitch_delta <= pitch_magnitude - 18'sd16128; pitch_slope <= 18'sd30; pitch_shift <= 4'd7;
                        end else if (pitch_magnitude >= 18'sd15872) begin
                            pitch_base <= 16'sd756; pitch_delta <= pitch_magnitude - 18'sd15872; pitch_slope <= 18'sd43; pitch_shift <= 4'd8;
                        end else if (pitch_magnitude >= 18'sd15360) begin
                            pitch_base <= 16'sd696; pitch_delta <= pitch_magnitude - 18'sd15360; pitch_slope <= 18'sd60; pitch_shift <= 4'd9;
                        end else if (pitch_magnitude >= 18'sd14336) begin
                            pitch_base <= 16'sd610; pitch_delta <= pitch_magnitude - 18'sd14336; pitch_slope <= 18'sd86; pitch_shift <= 4'd10;
                        end else if (pitch_magnitude >= 18'sd12288) begin
                            pitch_base <= 16'sd486; pitch_delta <= pitch_magnitude - 18'sd12288; pitch_slope <= 18'sd124; pitch_shift <= 4'd11;
                        end else if (pitch_magnitude >= 18'sd10240) begin
                            pitch_base <= 16'sd387; pitch_delta <= pitch_magnitude - 18'sd10240; pitch_slope <= 18'sd99; pitch_shift <= 4'd11;
                        end else if (pitch_magnitude >= 18'sd8192) begin
                            pitch_base <= 16'sd300; pitch_delta <= pitch_magnitude - 18'sd8192; pitch_slope <= 18'sd87; pitch_shift <= 4'd11;
                        end else if (pitch_magnitude >= 18'sd6144) begin
                            pitch_base <= 16'sd220; pitch_delta <= pitch_magnitude - 18'sd6144; pitch_slope <= 18'sd80; pitch_shift <= 4'd11;
                        end else if (pitch_magnitude >= 18'sd4096) begin
                            pitch_base <= 16'sd145; pitch_delta <= pitch_magnitude - 18'sd4096; pitch_slope <= 18'sd75; pitch_shift <= 4'd11;
                        end else if (pitch_magnitude >= 18'sd2048) begin
                            pitch_base <= 16'sd72; pitch_delta <= pitch_magnitude - 18'sd2048; pitch_slope <= 18'sd73; pitch_shift <= 4'd11;
                        end else begin
                            pitch_base <= 0; pitch_delta <= pitch_magnitude; pitch_slope <= 18'sd72; pitch_shift <= 4'd11;
                        end
                        cordic_iter <= 0;
                        if (rot22 < 0) begin
                            cordic_x <= -{{2{rot22[17]}}, rot22};
                            cordic_y <= -((2 * (temp_product + product)) >>> 14);
                            cordic_angle <= (temp_product + product) >= 0 ? 1800 : -1800;
                        end else begin
                            cordic_x <= {{2{rot22[17]}}, rot22};
                            cordic_y <= (2 * (temp_product + product)) >>> 14;
                            cordic_angle <= 0;
                        end
                        state <= ROLL_ANGLE;
                    end
                endcase
            end else begin
                case (step)
                    15: omega_x <= limit18(omega_base_pending + gyro_corr_pending);
                    16: omega_y <= limit18(omega_base_pending + gyro_corr_pending);
                    17: omega_z <= limit18(omega_base_pending + gyro_corr_pending);
                endcase
                phase <= 2'd0;
                step <= step + 1'b1;
            end
            ROLL_ANGLE: if (cordic_iter < 14) begin
                if (cordic_iter == 0) begin
                    mul_a <= pitch_delta;
                    mul_b <= pitch_slope;
                end else if (cordic_iter == 1) begin
                    pitch_product <= product;
                end else if (cordic_iter == 2) begin
                    if (pitch_negative)
                        pitch_deg_x10 <= -(pitch_base + (pitch_product >>> pitch_shift));
                    else
                        pitch_deg_x10 <= pitch_base + (pitch_product >>> pitch_shift);
                end
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
            end else begin
                roll_pending <= cordic_angle;
                cordic_iter <= 0;
                if (rot00 < 0) begin
                    cordic_x <= -{{2{rot00[17]}}, rot00};
                    cordic_y <= -{{2{rot01[17]}}, rot01};
                    cordic_angle <= rot01 >= 0 ? 1800 : -1800;
                end else begin
                    cordic_x <= {{2{rot00[17]}}, rot00};
                    cordic_y <= {{2{rot01[17]}}, rot01};
                    cordic_angle <= 0;
                end
                state <= YAW_ANGLE;
            end
            YAW_ANGLE: if (cordic_iter < 14) begin
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
            end else begin
                roll_deg_x10 <= roll_pending;
                yaw_deg_x10 <= cordic_angle;
                q0 <= n0; q1 <= n1; q2 <= n2; q3 <= n3;
                attitude_valid <= 1'b1;
                state <= IDLE;
            end
            default: state <= IDLE;
        endcase
    end
end

endmodule
