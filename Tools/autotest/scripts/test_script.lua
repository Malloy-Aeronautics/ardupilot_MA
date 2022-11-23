tick_interval = 10
debug_mode = true


min_voltage = 4.6
max_vibe_x = 2.0
max_vibe_y = 2.0
max_vibe_z = 5.0
max_roll_variance = 25.0
max_roll_variance = 25.0
max_pitch_variance = 25.0
max_yaw_variance = 25.0
n_sats_min = 25
h_dop_max = 100.0
delta_ms_max = 5000.0
delta_pd_max = 1.0
vel_var_max = 10.0
pos_var_max = 10.0
hgt_var_max = 10.0
mag_var_max = 10.0
tas_var_max = 10.0


debug_output = ""


function print_debug(msg)
	if (debug_mode == true) then
		gcs:send_text(6, "[MA_Script] " .. msg)
	end
end


function warning_to_gcs(msg)
	gcs:send_text(1, "[MA_Script] " .. msg)
end


function sizeof(buff)
	local count = 0
	for i in ipairs(buff) do
		count = count + 1
	end
	return count
end


--from libraries/AP_Math/AP_Math.cpp:
--
--float wrap_360_cd(const float angle)
--{
--    float res = fmodf(angle, 36000.0f);
--    if (res < 0) {
--        res += 36000.0f;
--    }
--    return res;
--}


function wrap_360_cd(angle)
	local res = math.fmod(angle, 36000.0)
	if (res < 0.0) then
		res = res + 36000.0
	end
	return res
end


voltage_buffer = { }
voltage_len = 10
voltage_end = 1


function put_voltage()
	if not (analog == nil) then
		voltage_buffer[voltage_end] = analog:board_voltage()
		voltage_end = voltage_end + 1
		if (voltage_end > voltage_len) then
			voltage_end = 1
		end
	end
end


function check_average_voltage()
	result = 0
	count = sizeof(voltage_buffer)
	if (count == 0) then
		return
	end
	for i = 1, count do
		result = result + voltage_buffer[i]
	end
	result = result / count
	if (result < min_voltage) then
		warning_to_gcs("Board voltage low: " .. result .. "V")
	end
end


vibe_buffer = { }
vibe_end = 1
vibe_1_buffer = { }
vibe_1_end = 1
vibe_2_buffer = { }
vibe_2_end = 1
vibe_len = 10


function put_vibe()
	if not (ins == nil) then
		local n_accel = ins:get_accel_count()
		for id = 1, n_accel do
			local vibe = ins:get_vibration_levels(id - 1)
			if (id == 1) then
				vibe_1_buffer[vibe_1_end] = vibe
				vibe_1_end = vibe_1_end + 1
				if (vibe_1_end > vibe_len) then
					vibe_1_end = 1
				end
			end
			if (id == 2) then
				vibe_2_buffer[vibe_2_end] = vibe
				vibe_2_end = vibe_2_end + 1
				if (vibe_2_end > vibe_len) then
					vibe_2_end = 1
				end
			end
		end
	else
		if not (ahrs == nil) then
			local vibe = ahrs:get_vibration()
			vibe_buffer[vibe_end] = vibe
			vibe_end = vibe_end + 1
			if (vibe_end > vibe_len) then
				vibe_end = 1
			end
		end
	end
end


function check_average_vibe()
	if not (ins == nil) then
		local n_accel = ins:get_accel_count()
		for id = 1, n_accel do
			local result_x = 0
			local result_y = 0
			local result_z = 0
			count = sizeof(vibe_1_buffer)
			if (count == 0) then
				return
			end
			for i = 1, count do
				if (id == 1) then
					result_x = result_x + vibe_1_buffer[i]:x()
					result_y = result_y + vibe_1_buffer[i]:y()
					result_z = result_z + vibe_1_buffer[i]:z()
				end
				if (id == 2) then
					result_x = result_x + vibe_2_buffer[i]:x()
					result_y = result_y + vibe_2_buffer[i]:y()
					result_z = result_z + vibe_2_buffer[i]:z()
				end
			end
			result_x = result_x / count
			result_y = result_y / count
			result_z = result_z / count
			if (result_x > max_vibe_x) then
				warning_to_gcs("Vibe-X-" .. id .. " high: " .. result_x)
			end
			if (result_y > max_vibe_y) then
				warning_to_gcs("Vibe-Y-" .. id .. " high: " .. result_y)
			end
			if (result_z > max_vibe_z) then
				warning_to_gcs("Vibe-Z-" .. id .. " high: " .. result_z)
			end			
		end
	else
		if not (ahrs == nil) then
			local result_x = 0
			local result_y = 0
			local result_z = 0
			count = sizeof(vibe_buffer)
			if (count == 0) then
				return
			end
			for i = 1, count do
				result_x = result_x + vibe_buffer[i]:x()
				result_y = result_y + vibe_buffer[i]:y()
				result_z = result_z + vibe_buffer[i]:z()
			end
			result_x = result_x / count
			result_y = result_y / count
			result_z = result_z / count
			if (result_x > max_vibe_x) then
				warning_to_gcs("Vibe-X high: " .. result_x)
			end
			if (result_y > max_vibe_y) then
				warning_to_gcs("Vibe-Y high: " .. result_y)
			end
			if (result_z > max_vibe_z) then
				warning_to_gcs("Vibe-Z high: " .. result_z)
			end
		end
	end
end


function check_attitude()
	if not (attitude_control_multi == nil) then
		local target_att = attitude_control_multi:get_att_target_euler_cd()	
		warning_to_gcs("target_roll: " .. target_att:x() .. " target_pitch: " .. target_att:y() .. " target_yaw: " .. wrap_360_cd(target_att:z())) 
	else
		warning_to_gcs("attitude_control was nil...")
	end
end


function check_gps()
	if not (gps == nil) then
		local num_sensors = gps:num_sensors()
		for id = 1, num_sensors do
			local n_sats = gps:num_sats(id - 1)
			print_debug("NSats-" .. id .. ": " .. n_sats)
			local h_dop = gps:get_hdop(id - 1)
			print_debug("HDop-" .. id .. ": " .. h_dop)
			local speed = gps:ground_speed(id - 1)
			print_debug("Speed-" .. id .. ": " .. speed)
			local delta_ms = gps:last_message_delta_time_ms(id - 1)
			print_debug("GPA_delta-" .. id .. ": " .. delta_ms .. "ms")
			if (n_sats < n_sats_min) then
				warning_to_gcs("NSats was under the threshold of " .. n_sats_min .. " (" .. n_sats .. ")")
			end
			if (h_dop > h_dop_max) then
				warning_to_gcs("HDop was over the threshold of " .. h_dop_max .. " (" .. h_dop .. ")")
			end
			if (delta_ms > delta_ms_max) then
				warning_to_gcs("GPA_delta was over the threshold of " .. delta_ms_max .. "ms (" .. delta_ms .. "ms)")
			end
		end
	else
		warning_to_gcs("gps was nil...")
	end
end


function check_xkf1()
	if not (NavEKF2() == nil) then
		NavEKF2_ud = NavEKF2()
		local ekf2_healthy = NavEKF2_ud:healthy()
		--report_ekf_healthy(2, healthy)
		local num_ekf2_cores = NavEKF2_ud:activeCores()
		--report_ekf_cores(2, num_ekf2_cores)
		local primary_id_2 = NavEKF2_ud:getPrimaryCoreIMUIndex()
		--report_ekf_primary_id(2, primary_id_2)
		
		if (num_ekf2_cores == 2) then
			local pd_0 = 0.0
			NavEKF2_ud:getPosD(0, pd_0)
			local pd_1 = 0.0
			NavEKF2_ud:getPosD(1, pd_1)
			print_debug("pd-0: " .. pd_0 .. " | pd-1: " .. pd_1)
			if (math.abs(pd_1 - pd_0) > delta_pd_max) then
				warning_to_gcs("XKF1.PD delta > " .. delta_pd_max)
			end
		end
	else
		warning_to_gcs("EKF2 not found!")
	end
	
	if not (NavEKF3() == nil) then
		NavEKF3_ud = NavEKF3()
		local ekf3_healthy = NavEKF3_ud:healthy()
		--report_ekf_healthy(3, healthy)
		local num_ekf3_cores = NavEKF3_ud:activeCores()
		--report_ekf_cores(3, num_ekf3_cores)
		local primary_id_3 = NavEKF3_ud:getPrimaryCoreIMUIndex()
		--report_ekf_primary_id(3, primary_id_3)
	else
		warning_to_gcs("EKF3 not found!")
	end
end


function check_posd()
	if not (AP_AHRS_NavEKF() == nil) then
		AP_AHRS_NavEKF_ud = AP_AHRS_NavEKF()
		local delta_posd = AP_AHRS_NavEKF_ud:get_delta_posd()
		warning_to_gcs("delta_posd: " .. delta_posd)
	end
end


function check_num_ekf2_cores()
	if not (NavEKF2() == nil) then
		NavEKF2_ud = NavEKF2()
		local num_ekf2_cores = NavEKF2_ud:activeCores()
		warning_to_gcs("Active EKF2 cores: " .. num_ekf2_cores)
	end
end


function check_num_ekf3_cores()
	if not (NavEKF3() == nil) then
		NavEKF3_ud = NavEKF3()
		local num_ekf3_cores = NavEKF3_ud:activeCores()
		warning_to_gcs("Active EKF3 cores: " .. num_ekf3_cores)
	end
end


function ekf_test()
	if not (ahrs == nil) then
		local n_ekf3_cores = ahrs:get_num_ekf3_cores()
		warning_to_gcs("Active EKF3 cores: " .. n_ekf3_cores)
		if ahrs:is_ekf3_healthy() then
			warning_to_gcs("EKF3 is healthy!")
		else
			warning_to_gcs("EKF3 is NOT healthy!")
		end
		local d_posd = ahrs:get_delta_posd()
		warning_to_gcs("Delta posD: " .. d_posd)
	else
		warning_to_gcs("AHRS was nil...")
	end
end


vel_1_buffer = { }
vel_2_buffer = { }
vel_1_end = 1
vel_2_end = 1
vel_len = 10
pos_1_buffer = { }
pos_2_buffer = { }
pos_1_end = 1
pos_2_end = 1
pos_len = 10
hgt_1_buffer = { }
hgt_2_buffer = { }
hgt_1_end = 1
hgt_2_end = 1
hgt_len = 10
mag_1_buffer = { }
mag_2_buffer = { }
mag_1_end = 1
mag_2_end = 1
mag_len = 10
tas_1_buffer = { }
tas_2_buffer = { }
tas_1_end = 1
tas_2_end = 1
tas_len = 10


function put_xkf4()
	local sc = 100.0
	if not (ahrs == nil) then
		local n_ekf3_cores = ahrs:get_num_ekf3_cores()
		if (n_ekf3_cores == 2) then
			for id = 1, 2 do
				vel_var, pos_var, hgt_var, mag_var, tas_var, offset = ahrs:get_ekf3_variances(id - 1)
				local tmp_var = math.max(math.max(mag_var:x(), mag_var:y()), mag_var:z())
				if (id == 1) then
					vel_1_buffer[vel_1_end] = math.floor(vel_var * sc)
					vel_1_end = vel_1_end + 1
					if (vel_1_end > vel_len) then
						vel_1_end = 1
					end
					pos_1_buffer[pos_1_end] = math.floor(pos_var * sc)
					pos_1_end = pos_1_end + 1
					if (pos_1_end > pos_len) then
						pos_1_end = 1
					end
					hgt_1_buffer[hgt_1_end] = math.floor(hgt_var * sc)
					hgt_1_end = hgt_1_end + 1
					if (hgt_1_end > hgt_len) then
						hgt_1_end = 1
					end
					mag_1_buffer[mag_1_end] = math.floor(tmp_var * sc)
					mag_1_end = mag_1_end + 1
					if (mag_1_end > mag_len) then
						mag_1_end = 1
					end
					tas_1_buffer[tas_1_end] = math.floor(tas_var * sc)
					tas_1_end = tas_1_end + 1
					if (tas_1_end > tas_len) then
						tas_1_end = 1
					end
				end
				if (id == 2) then
					vel_2_buffer[vel_2_end] = math.floor(vel_var * sc)
					vel_2_end = vel_2_end + 1
					if (vel_2_end > vel_len) then
						vel_2_end = 1
					end
					pos_2_buffer[pos_2_end] = math.floor(pos_var * sc)
					pos_2_end = pos_2_end + 1
					if (pos_2_end > pos_len) then
						pos_2_end = 1
					end
					hgt_2_buffer[hgt_2_end] = math.floor(hgt_var * sc)
					hgt_2_end = hgt_2_end + 1
					if (hgt_2_end > hgt_len) then
						hgt_2_end = 1
					end
					mag_2_buffer[mag_2_end] = math.floor(tmp_var * sc)
					mag_2_end = mag_2_end + 1
					if (mag_2_end > mag_len) then
						mag_2_end = 1
					end
					tas_2_buffer[tas_2_end] = math.floor(tas_var * sc)
					tas_2_end = tas_2_end + 1
					if (tas_2_end > tas_len) then
						tas_2_end = 1
					end
				end
			end
		end	
	end
end


function check_average_exkf4()
	if not (ahrs == nil) then
		local n_ekf3_cores = ahrs:get_num_ekf3_cores()
		if (n_ekf3_cores == 2) then
			for id = 1, 2 do
				local vel_result = 0
				local pos_result = 0
				local hgt_result = 0
				local mag_result = 0
				local tas_result = 0
				local count = sizeof(vel_1_buffer)
				if (count == 10) then
					for i = 1, 10 do
						if (id == 1) then
							vel_result = vel_result + vel_1_buffer[i]
							pos_result = pos_result + pos_1_buffer[i]
							hgt_result = hgt_result + hgt_1_buffer[i]
							mag_result = mag_result + mag_1_buffer[i]
							tas_result = tas_result + tas_1_buffer[i]
						end
						if (id == 2) then
							vel_result = vel_result + vel_2_buffer[i]
							pos_result = pos_result + pos_2_buffer[i]
							hgt_result = hgt_result + hgt_2_buffer[i]
							mag_result = mag_result + mag_2_buffer[i]
							tas_result = tas_result + tas_2_buffer[i]
						end
					end	
					vel_result = vel_result / count
					pos_result = pos_result / count
					hgt_result = hgt_result / count
					mag_result = mag_result / count
					tas_result = tas_result / count
					warning_to_gcs("Vel. variance " .. id .. ": " .. vel_result)
					warning_to_gcs("Pos. variance " .. id .. ": " .. pos_result)
					warning_to_gcs("Hgt. variance " .. id .. ": " .. hgt_result)
					warning_to_gcs("Mag. variance " .. id .. ": " .. mag_result)
					warning_to_gcs("Tas. variance " .. id .. ": " .. tas_result)
					if (vel_result > vel_var_max) then
						warning_to_gcs("Vel. variance " .. id .. " high: " .. vel_result)
					end
					if (pos_result > pos_var_max) then
						warning_to_gcs("Pos. variance " .. id .. " high: " .. pos_result)
					end
					if (hgt_result >hgt_var_max) then
						warning_to_gcs("Hgt. variance " .. id .. " high: " .. hgt_result)
					end
					if (mag_result > mag_var_max) then
						warning_to_gcs("Mag. variance " .. id .. " high: " .. mag_result)
					end
					if (tas_result > tas_var_max) then
						warning_to_gcs("Tas. variance " .. id .. " high: " .. tas_result)
					end
				else
					warning_to_gcs("XKF4 buffer size: " .. count)
				end
			end
		end
	else
		warning_to_gcs("AHRS was nil...")
	end
end


function run_50Hz_loop()
	debug_output = debug_output .. "-"
end


function run_20Hz_loop()
	debug_output = debug_output .. "^"
end


function run_10Hz_loop()
	debug_output = debug_output .. "*"
	put_voltage()
	put_vibe()
	put_xkf4()
end


function run_5Hz_loop()
	debug_output = debug_output .. "+"
end


function run_2Hz_loop()
	debug_output = debug_output .. "o"
	--check_xkf1()
	--check_posd()
end


function run_1Hz_loop()
	debug_output = debug_output .. "O"
	--check_num_ekf2_cores()
	--check_num_ekf3_cores()
	--ekf_test()
	check_average_voltage()
	check_average_vibe()
	check_average_exkf4()
	--check_attitude()
end


cycle_count = 0
function update()

	--if (math.fmod(cycle_count, 2) == 0) then
	--	run_50Hz_loop()
	--end

	--if (math.fmod(cycle_count, 5) == 0) then
	--	run_20Hz_loop()
	--end

	if (math.fmod(cycle_count, 10) == 0) then
		run_10Hz_loop()
	end

	if (math.fmod(cycle_count, 20) == 0) then
		run_5Hz_loop()
	end

	if (math.fmod(cycle_count, 50) == 0) then
		run_2Hz_loop()
	end
	
	if (math.fmod(cycle_count, 100) == 0) then
		run_1Hz_loop()
	end
	
	cycle_count = cycle_count + 1

	if (cycle_count >= 100) then
		if (debug_mode == true) then
			gcs:send_text(6, debug_output)
		end
		debug_output = ""
		cycle_count = 0
	end

	return update, tick_interval

end


return update()
