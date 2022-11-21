tick_interval = 10
debug_mode = true


min_voltage = 4.6
max_vibe_x = 5.0
max_vibe_y = 5.0
max_vibe_z = 10.0
max_roll_variance = 25.0
max_roll_variance = 25.0
max_pitch_variance = 25.0
max_yaw_variance = 25.0
n_sats_min = 25
h_dop_max = 100.0
delta_ms_max = 5000.0
delta_pd_max = 1.0


debug_output = ""


function print_debug(msg)
	if (debug_mode == true) then
		gcs:send_text(6, "[MA_Script] " .. msg)
	end
end


function warning_to_gcs(msg)
	gcs:send_text(1, "[MA_Script] " .. msg)
end


function sizeof(arr)
	local count = 0
	for i in ipairs(arr) do
		count = count + 1
	end
	return count
end


voltage_array = { }
voltage_len = 20
voltage_end = 1


function put_voltage(v)
	voltage_array[voltage_end] = v
	voltage_end = voltage_end + 1
	if (voltage_end > voltage_len) then
		voltage_end = 1
	end
end


function check_average_voltage()
	result = 0
	if not (analog == nil) then
		put_voltage(analog:board_voltage())
		count = sizeof(voltage_array)
		if (count == 0) then
			return result
		end
		for i = 1, count do
			result = result + voltage_array[i]
		end
		return (result / count)
	end
	return result
end


function check_voltage()
	local average_u = check_average_voltage()
	if (average_u < min_voltage) then
		warning_to_gcs("Board voltage: " .. average_u .. "V")
	end
end


function check_vibe()
	if not (ins == nil) then
		local n_accel = ins:get_accel_count()
		for id = 1, n_accel do
			local vibe = ins:get_vibration_levels(id - 1)
			if (vibe:x() > max_vibe_x) then
				warning_to_gcs("VibeX-" .. id .." > " .. max_vibe_x .. " (" .. vibe:x() .. ")")
			end
			if (vibe:y() > max_vibe_y) then
				warning_to_gcs("VibeY-" .. id .." > " .. max_vibe_y .. " (" .. vibe:y() .. ")")
			end
			if (vibe:z() > max_vibe_z) then
				warning_to_gcs("VibeZ-" .. id .." > " .. max_vibe_z .. " (" .. vibe:z() .. ")")
			end
		end
	else
		if not (ahrs == nil) then
			local vibe = ahrs:get_vibration()
			if (vibe:x() > max_vibe_x) then
				warning_to_gcs("VibeX > " .. max_vibe_x .. " (" .. vibe:x() .. ")")
			end
			if (vibe:y() > max_vibe_y) then
				warning_to_gcs("VibeY > " .. max_vibe_y .. " (" .. vibe:y() .. ")")
			end
			if (vibe:z() > max_vibe_z) then
				warning_to_gcs("VibeZ > " .. max_vibe_z .. " (" .. vibe:z() .. ")")
			end
		end
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
		warning_to_gcs("num_ekf3_cores: " .. n_ekf3_cores)
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
	check_vibe()
end


function run_5Hz_loop()
	debug_output = debug_output .. "+"
end


function run_2Hz_loop()
	debug_output = debug_output .. "o"
	--check_xkf1()
	--check_posd()
	--check_voltage()
end


function run_1Hz_loop()
	--check_num_ekf2_cores()
	--check_num_ekf3_cores()
	ekf_test()
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
