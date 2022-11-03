min_voltage = 4.5
max_vibe_x = 5.0
max_vibe_y = 5.0
max_vibe_z = 30.0
max_roll_variance = 25.0
max_roll_variance = 25.0
max_pitch_variance = 25.0
max_yaw_variance = 25.0
n_sats_min = 25
h_dop_max = 100.0
delta_ms_max = 5000.0
delta_pd_max = 1.0

lat_prev = 0
lng_prev = 0


tick_interval = 500
cycle_count = 0;


aircraft_top_speed = 25.0
max_ds_per_tick_interval = aircraft_top_speed * tick_interval / 1000.0


debug_mode = true
function debug_message_to_gcs(msg)
	if (debug_mode == true) then
		if (math.fmod(cycle_count, 20) == 0) then
			gcs:send_text(6, "[MA_Script] " .. msg)
		end
	end
end


function message_to_gcs(msg)
	gcs:send_text(4, "[MA_Script] " .. msg)
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


function mean(values)
	result = 0
	count = sizeof(values)
	if (count == 0) then
		return result
	end
	for i = 1, count do
		result = result + values[i]
	end
	result = result / count
	return result
end


function cross_correlation(series_a, series_b)
	result = 0
	if not (sizeof(series_a) == sizeof(series_b)) then
		return result
	end
	ma = mean(series_a)
	mb = mean(series_b)
	numerator = 0
	count = sizeof(series_a)
	for i = 1, count do
		numerator = numerator + ((series_a[i] - ma) * (series_b[i] - mb))
	end
	denominator_a = 0
	denominator_b = 0
	for j = 1, count do
		denominator_a = denominator_a + ((series_a[j] - ma) * (series_a[j]-ma))
		denominator_b = denominator_b + ((series_b[j] - ma) * (series_b[j]-mb))
	end
	denominator = math.sqrt(denominator_a) * math.sqrt(denominator_b)
	if (denominator == 0) then
		return result
	end
	result = numerator / denominator
	return result
end


function to_deg(value)
	d = value * 180.0 / 3.1416
	return d
end


function to_rad(value)
	r = value * 3.1416 / 180.0
	return r
end


function angle_variance_deg(ang_a, ang_b)
	a = ang_a + 360.0
	b = ang_b + 360.0
	diff = math.abs(a - b)
	while (diff > 180) do
		diff = diff - 180
	end
	percentage = diff / 180.0 * 100.0
	return percentage
end


function dist(lat_1, lng_1, lat_2, lng_2)
	d = (math.acos(math.sin(to_rad(lat_1)) * math.sin(to_rad(lat_2)) + math.cos(to_rad(lat_1)) * math.cos(to_rad(lat_2)) * math.cos(to_rad(lng_2 - lng_1))) * 6371000.0)
	return d
end 


function do_sanity_check()
	cycle_count = cycle_count + 1
	if (math.fmod(cycle_count, 60) == 0) then
		warning_to_gcs("Lua script is running...")
	end
	if (cycle_count >= 600) then
		cycle_count = 0;
		--warning_to_gcs("Lua script is running...")
	end
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


vibe_array = { }
vibe_len = 20
vibe_end = 1


--function put_vibe(v)
--	vibe_array[vibe_end] = v
--	vibe_end = vibe_end + 1
--	if (vibe_end > vibe_len) then
--		vibe_end = 1
--	end
--end
--
--
--function check_average_vibe_x()
--	result = 0
--	if not (ins == nil) then
--		put_vibe(analog:board_voltage())
--		count = sizeof(voltage_array)
--		if (count == 0) then
--			return result
--		end
--		for i = 1, count do
--			result = result + voltage_array[i]
--		end
--		return (result / count)
--	end
--	return result
--end


function check_vibe()
	if not (ins == nil) then
		local n_accel = ins:get_accel_count()
		for id = 1, n_accel do
			local vibe = ins:get_vibration_levels(id - 1)
			debug_message_to_gcs("Vibe-" .. id .. ": " .. vibe:x() .. " | " .. vibe:y() .. " | " .. vibe:z())	
			if (vibe:x() > max_vibe_x) then
				warning_to_gcs("VibeX-" .. id .." was over the threshold of " .. max_vibe_x .. " (" .. vibe:x() .. ")")
			end
			if (vibe:y() > max_vibe_y) then
				warning_to_gcs("VibeY-" .. id .." was over the threshold of " .. max_vibe_y .. " (" .. vibe:y() .. ")")
			end
			if (vibe:z() > max_vibe_z) then
				warning_to_gcs("VibeZ-" .. id .." was over the threshold of " .. max_vibe_z .. " (" .. vibe:z() .. ")")
			end
		end
	end
end


function check_attitude()
	local achieved_roll = 0.0
	local achieved_pitch = 0.0
	local achieved_yaw = 0.0
	local target_roll = 0.0
	local target_pitch = 0.0
	local target_yaw = 0.0
	if not (ahrs == nil) then
		achieved_roll = to_deg(ahrs:get_roll())
		achieved_pitch = to_deg(ahrs:get_pitch())
		achieved_yaw = to_deg(ahrs:get_yaw())
		debug_message_to_gcs("Roll: " .. achieved_roll .. " Pitch: " .. achieved_pitch .. " Yaw: " .. achieved_yaw)
	end
	if not (AC_AttitudeControl == nil) then
		target_roll = to_deg(AC_AttitudeControl:get_att_target_euler_rad():x())
		target_pitch = to_deg(AC_AttitudeControl:get_att_target_euler_rad():y())
		target_yaw = to_deg(AC_AttitudeControl:get_att_target_euler_rad():z())
	end
	local roll_var = angle_variance_deg(achieved_roll, target_roll)
	local pitch_var = angle_variance_deg(achieved_pitch, target_pitch)
	local yaw_var = angle_variance_deg(achieved_yaw, target_yaw)
	if (roll_var > max_roll_variance) then
		warning_to_gcs("Roll variance: " .. roll_var .. "%")
	end
	if (pitch_var > max_pitch_variance) then
		warning_to_gcs("Pitch variance: " .. pitch_var .. "%")
	end
	if (yaw_var > max_yaw_variance) then
		warning_to_gcs("Yaw variance: " .. yaw_var .. "%")
	end
end


function check_altitude()
	local target = Location()
	if not (target == nil) then
		local alt_origin = target:origin_alt()
		local alt_terrain = target:terrain_alt()
		local alt_relative = target:relative_alt()
		local alt_baro = baro:get_altitude()
		debug_message_to_gcs("Alt-Barometer: " .. alt_baro .. " Alt-Origin: " .. alt_origin .. " Alt-Terrain: " .. alt_terrain .. " Alt-Relative: " .. alt_relative)
	end
end


function check_barometric_altitude()
	-- H = 44330.0 * (1.0 - (P/P0)^(1.0/5.255))
	-- P0 = 1013.25hPa
	local press = baro:get_pressure() * 0.01
	local calculated_alt = 44330.0 * (1.0 - (press / 1013.25) ^ 0.19)
	local barometer_alt = baro:get_altitude()
	--warning_to_gcs("Delta Alt: " .. calculated_alt - barometer_alt)
end


gps_cores_reported = false


function report_gps_cores(n)
	--if not (gps_cores_reported == true) then
	if (math.fmod(cycle_count, 20) == 0) then
		--gps_cores_reported = true
		message_to_gcs("GPS cores found: " .. n) 
	end
end



function check_gps()
	if not (gps == nil) then
		local num_sensors = gps:num_sensors()
		report_gps_cores(num_sensors)
		--debug_message_to_gcs(5, "Number of GPS sensors: " .. num_sensors)
		for id = 1, num_sensors do
			local n_sats = gps:num_sats(id - 1)
			debug_message_to_gcs("NSats-" .. id .. ": " .. n_sats)
			local h_dop = gps:get_hdop(id - 1)
			debug_message_to_gcs("HDop-" .. id .. ": " .. h_dop)
			local speed = gps:ground_speed(id - 1)
			debug_message_to_gcs("Speed-" .. id .. ": " .. speed)
			local delta_ms = gps:last_message_delta_time_ms(id - 1)
			debug_message_to_gcs("GPA_delta-" .. id .. ": " .. delta_ms .. "ms")
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


rcou_array = { }
num_motors = 8


function check_rcou()
	if not (rcout == nil) then
		local rcou_acc = 0.0
		for m_id = 1, num_motors do
			rcou_array[m_id] = rcout:read(m_id - 1)
			rcou_acc = rcou_acc + rcou_array[m_id]
		end
		local rcou_average = rcou_acc / num_motors;
		for m_id = 1, num_motors do
			if (math.abs(rcou_array[m_id] - rcou_acc) > max_rcou_diff) then
				warning_to_gcs("RCOU-" .. m_id .. ": " .. rcou_array[m_id])
			end
		end
		
		--local rcou_0 = rcout:read(0)
		--local rcou_1 = rcout:read(1)
		--local rcou_2 = rcout:read(2)
		--local rcou_3 = rcout:read(3)
		--local rcou_4 = rcout:read(4)
		--local rcou_5 = rcout:read(5)
		--local rcou_6 = rcout:read(6)
		--local rcou_7 = rcout:read(7)
		--message_to_gcs("RCOU-0 " .. rcou_0 .. " RCOU-1 " .. rcou_1 .. " RCOU-2 " .. rcou_2 .. " RCOU-3 " .. rcou_3)
		--message_to_gcs("RCOU-4 " .. rcou_4 .. " RCOU-5 " .. rcou_5 .. " RCOU-6 " .. rcou_6 .. " RCOU-7 " .. rcou_7)
	end
end


ekf2_reported = false
ekf2_cores_reported = false
ekf3_reported = false
ekf3_cores_reported = false


function report_ekf2_found()
	--if not (ekf2_reported == true) then
	if (math.fmod(cycle_count, 20) == 0) then
		--ekf2_reported = true
		message_to_gcs("EKF2 found!")
	end
end


function report_ekf2_healthy()
	if (math.fmod(cycle_count, 20) == 0) then
		local is_ekf_healthy = NavEKF2_ud:healthy()
		if (is_ekf_healthy == true) then
			message_to_gcs("EKF2 is healthy.");
		else
			message_to_gcs("EKF2 is not healthy.");
		end
	end
end


function report_ekf2_cores(n)
	--if not (ekf2_cores_reported == true) then
	if (math.fmod(cycle_count, 20) == 0) then
		--ekf2_cores_reported = true
		message_to_gcs("EKF2 cores found: " .. n) 
	end
end


function report_ekf2_primary_id(id)
	if (math.fmod(cycle_count, 20) == 0) then
		message_to_gcs("EKF2 primary core id: " .. id)
	end	
end


function report_ekf3_found()
	--if not (ekf3_reported == true) then
	if (math.fmod(cycle_count, 20) == 0) then
		--ekf3_reported = true
		message_to_gcs("EKF3 found!")
	end
end


function report_ekf3_healthy()
	if (math.fmod(cycle_count, 20) == 0) then
		local is_ekf_healthy = NavEKF3_ud:healthy()
		if (is_ekf_healthy == true) then
			message_to_gcs("EKF3 is healthy.");
		else
			message_to_gcs("EKF3 is not healthy.");
		end
	end
end


function report_ekf3_cores(n)
	--if not (ekf3_cores_reported == true) then
	if (math.fmod(cycle_count, 20) == 0) then
		--ekf3_cores_reported = true
		message_to_gcs("EKF3 cores found: " .. n) 
	end
end


function report_ekf3_primary_id(id)
	if (math.fmod(cycle_count, 20) == 0) then
		message_to_gcs("EKF3 primary core id: " .. id)
	end	
end


function check_xkf1()
	if not (NavEKF2() == nil) then
		--report_ekf2_found()
		report_ekf2_healthy()
		NavEKF2_ud = NavEKF2()
		local num_ekf2_cores = NavEKF2_ud:activeCores()
		report_ekf2_cores(num_ekf2_cores)

		local primary_id_2 = NavEKF2_ud:getPrimaryCoreIMUIndex()
		report_ekf2_primary_id(primary_id_2)
		--message_to_gcs("EKF2 primary core IMU ID: " .. primary_id) 
		
		if (num_ekf2_cores == 2) then
			local pd_0 = 0.0
			NavEKF2_ud:getPosD(0, pd_0)
			local pd_1 = 0.0
			NavEKF2_ud:getPosD(1, pd_1)
			warning_to_gcs("pd-0: " .. pd_0 .. " | pd-1: " .. pd_1)
			if (math.abs(pd_1 - pd_0) > delta_pd_max) then
				warning_to_gcs("XKF1.PD delta > " .. delta_pd_max)
			end
		else
			--local pd_0 = 0.0
			--NavEKF2_ud:getPosD(primary_id, pd_0)
			--message_to_gcs("pd-0: " .. pd_0)
		end
	else
		warning_to_gcs("EKF2 not found!")
	end
	
	if not (NavEKF3() == nil) then
		--report_ekf3_found()
		report_ekf3_healthy()
		NavEKF3_ud = NavEKF3()
		local num_ekf3_cores = NavEKF3_ud:activeCores()
		report_ekf3_cores(num_ekf3_cores)

		local primary_id_3 = NavEKF3_ud:getPrimaryCoreIMUIndex()
		report_ekf3_primary_id(primary_id_3)
		--message_to_gcs("EKF3 primary core IMU ID: " .. primary_id) 
	else
		warning_to_gcs("EKF3 not found!")
	end
end


function check_xkf4()
	if not (ahrs == nil) then
		local primary_id = ahrs:get_primary_core_index()
		debug_message_to_gcs("AHRS primary core index: " .. primary_id)
		vel_variance, pos_variance, height_variance, mag_variance, airspeed_variance = ahrs:get_variances()
		if vel_variance then
			debug_message_to_gcs(string.format("Variances Pos:%.1f Vel:%.1f Hgt:%.1f Mag:%.1f", pos_variance, vel_variance, height_variance, mag_variance:length()))		
			if (vel_variance > 1.0) then
				warning_to_gcs("Vel. variance was over the threshold of 1.0 (" .. vel_variance .. ")")
			end
			if (pos_variance > 1.0) then
				warning_to_gcs("Pos. variance was over the threshold of 1.0 (" .. pos_variance .. ")")
			end
			if (height_variance > 1.0) then
				warning_to_gcs("Pos. variance was over the threshold of 1.0 (" .. height_variance .. ")")
			end
			if (airspeed_variance > 1.0) then
				warning_to_gcs("Pos. variance was over the threshold of 1.0 (" .. airspeed_variance .. ")")
			end
		else
			debug_message_to_gcs(string.format("Failed to retrieve variances"))
		end
	end
end


function check_pos()
	local target = Location()
	if not (target == nil) then
		local lat_curr = target:lat()
		local lng_curr = target:lng()
		ds = dist(lat_prev, lng_prev, lat_curr, lng_curr)
		--debug_message_to_gcs("ds: " .. ds)
		if (ds > max_ds_per_tick_interval) then
			warning_to_gcs("Speed calculated from lat-lng data was suspiciously high.")
		end
		lat_prev = lat_curr
		lng_prev = lng_curr
	end
end


function update()
	
	do_sanity_check();
	
	check_voltage()
	check_vibe()
	check_attitude()
	check_altitude()
	check_barometric_altitude()
	check_gps()
	check_rcou()
	check_xkf1()
	check_xkf4()
	check_pos()
	
	return update, tick_interval
	
end


return update()
