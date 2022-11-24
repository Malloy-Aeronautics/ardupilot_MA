tick_interval = 10
debug_mode = false


min_voltage = 4.6
max_vibe_x = 2.0
max_vibe_y = 2.0
max_vibe_z = 5.0
max_roll_variance = 1.25
max_pitch_variance = 1.25
max_yaw_variance = 2.5
min_nsats = 25.0
max_hdop = 100.0
max_gpa_delta = 5000.0
max_delta_posd = 1.0
max_vel_var = 10.0
max_pos_var = 10.0
max_hgt_var = 10.0
max_mag_var = 10.0
max_tas_var = 10.0
max_speed = 10.0


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


function wrap_360_cd(angle)
	local res = math.fmod(angle, 360.0)
	if (res < 0.0) then
		res = res + 360.0
	end
	return res
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
	d = (math.acos(math.sin(math.rad(lat_1)) * math.sin(math.rad(lat_2)) + math.cos(math.rad(lat_1)) * math.cos(math.rad(lat_2)) * math.cos(math.rad(lng_2 - lng_1))) * 6371000.0)
	return d
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
	result = 0.0
	count = sizeof(voltage_buffer)
	if (count == 0) then
		return
	end
	for i = 1, count do
		result = result + voltage_buffer[i]
	end
	result = result / count
	if (result < min_voltage) then
		warning_to_gcs("Board voltage < " .. min_voltage .. "V (" .. result .. "V)")
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
			local result_x = 0.0
			local result_y = 0.0
			local result_z = 0.0
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
				warning_to_gcs("Vibe-X-" .. id .. " > " .. max_vibe_x .. " (" .. result_x .. ")")
			end
			if (result_y > max_vibe_y) then
				warning_to_gcs("Vibe-Y-" .. id .. " > " .. max_vibe_y .. " (" .. result_y .. ")")
			end
			if (result_z > max_vibe_z) then
				warning_to_gcs("Vibe-Z-" .. id .. " > " .. max_vibe_z .. " (" .. result_z .. ")")
			end			
		end
	else
		if not (ahrs == nil) then
			local result_x = 0.0
			local result_y = 0.0
			local result_z = 0.0
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
				warning_to_gcs("Vibe-X > " .. max_vibe_x .. " (" .. result_x .. ")")
			end
			if (result_y > max_vibe_y) then
				warning_to_gcs("Vibe-Y > " .. max_vibe_y .. " (" .. result_y .. ")")
			end
			if (result_z > max_vibe_z) then
				warning_to_gcs("Vibe-Z > " .. max_vibe_z .. " (" .. result_z .. ")")
			end
		end
	end
end


delta_roll_buffer = { }
delta_roll_end = 1
delta_pitch_buffer = { }
delta_pitch_end = 1
delta_yaw_buffer = { }
delta_yaw_end = 1
att_len = 10


function put_attitude()
	if not (attitude_control_multi == nil) then
		local target_att = attitude_control_multi:get_att_target_euler_cd()	
		local des_roll = target_att:x() * 0.01
		local des_pitch = target_att:y() * 0.01
		local des_yaw = wrap_360_cd(target_att:z() * 0.01)
		if not (ahrs == nil) then
			local achieved_roll = math.deg(ahrs:get_roll())
			local achieved_pitch = math.deg(ahrs:get_pitch())
			local achieved_yaw = wrap_360_cd(math.deg(ahrs:get_yaw()))
			delta_roll_buffer[delta_roll_end] = angle_variance_deg(des_roll, achieved_roll)
			delta_roll_end = delta_roll_end + 1
			if (delta_roll_end > att_len) then
				delta_roll_end = 1
			end
			delta_pitch_buffer[delta_pitch_end] = angle_variance_deg(des_pitch, achieved_pitch)
			delta_pitch_end = delta_pitch_end + 1
			if (delta_pitch_end > att_len) then
				delta_pitch_end = 1
			end
			delta_yaw_buffer[delta_yaw_end] = angle_variance_deg(des_yaw, achieved_yaw)
			delta_yaw_end = delta_yaw_end + 1
			if (delta_yaw_end > att_len) then
				delta_yaw_end = 1
			end
		else
			warning_to_gcs("ahrs was nil...")
		end
	else
		warning_to_gcs("attitude_control_multi was nil...")
	end
end


function check_average_attitude()
	if not (ahrs == nil) then
		if not (attitude_control_multi == nil) then
			delta_roll_result = 0.0
			delta_pitch_result = 0.0
			delta_yaw_result = 0.0
			count = sizeof(delta_roll_buffer)
			if (count == 0) then
				return
			end
			for i = 1, count do
				delta_roll_result = delta_roll_result + delta_roll_buffer[i]
				delta_pitch_result = delta_pitch_result + delta_pitch_buffer[i]
				delta_yaw_result = delta_yaw_result + delta_yaw_buffer[i]
			end
			delta_roll_result = delta_roll_result / count
			delta_pitch_result = delta_pitch_result / count
			delta_yaw_result = delta_yaw_result / count
			print_debug("Roll delta: " .. delta_roll_result .. "%")
			print_debug("Pitch delta: " .. delta_pitch_result .. "%")
			print_debug("Yaw delta: " .. delta_yaw_result .. "%")
			if (delta_roll_result > max_roll_variance) then
				warning_to_gcs("Roll delta > 1.0% (" .. delta_roll_result .. "%)")
			end
			if (delta_pitch_result > max_pitch_variance) then
				warning_to_gcs("Pitch delta > 1.0% (" .. delta_pitch_result .. "%)")
			end
			if (delta_yaw_result > max_yaw_variance) then
				warning_to_gcs("Yaw delta > 1.0% (" .. delta_yaw_result .. "%)")
			end
		end
	end
end


nsats_1_buffer = { }
nsats_1_end = 1
hdop_1_buffer = { }
hdop_1_end = 1
gpa_1_buffer = { }
gpa_1_end = 1
nsats_2_buffer = { }
nsats_2_end = 1
hdop_2_buffer = { }
hdop_2_end = 1
gpa_2_buffer = { }
gpa_2_end = 1
gps_len = 10


function put_gps()
	if not (gps == nil) then
		local num_sensors = gps:num_sensors()
		if (num_sensors > 0) then
			for id = 1, num_sensors do
				if (id == 1) then
					nsats_1_buffer[nsats_1_end] = gps:num_sats(id - 1)
					nsats_1_end = nsats_1_end + 1
					if (nsats_1_end > gps_len) then
						nsats_1_end = 1
					end
					hdop_1_buffer[hdop_1_end] = gps:get_hdop(id - 1)
					hdop_1_end = hdop_1_end + 1
					if (hdop_1_end > gps_len) then
						hdop_1_end = 1
					end
					gpa_1_buffer[gpa_1_end] = gps:last_message_delta_time_ms(id - 1)
					gpa_1_end = gpa_1_end + 1
					if (gpa_1_end > gps_len) then
						gpa_1_end = 1
					end
				end
				if (id == 2) then
					nsats_2_buffer[nsats_2_end] = gps:num_sats(id - 1)
					nsats_2_end = nsats_2_end + 1
					if (nsats_2_end > gps_len) then
						nsats_2_end = 1
					end
					hdop_2_buffer[hdop_2_end] = gps:get_hdop(id - 1)
					hdop_2_end = hdop_2_end + 1
					if (hdop_2_end > gps_len) then
						hdop_2_end = 1
					end
					gpa_2_buffer[gpa_2_end] = gps:last_message_delta_time_ms(id - 1)
					gpa_2_end = gpa_2_end + 1
					if (gpa_2_end > gps_len) then
						gpa_2_end = 1
					end
				end
			end
		end
	else
		warning_to_gcs("gps was nil...")
	end
end


function check_average_gps()
	if not (gps == nil) then
		local num_sensors = gps:num_sensors()
		if (num_sensors > 0) then
			for id = 1, num_sensors do
				if (id == 1) then
					local nsats_result = 0
					local hdop_result = 0
					local gpa_result = 0
					local count = sizeof(nsats_1_buffer)
					if (count == 10) then
						for i = 1, 10 do
							nsats_result = nsats_result + nsats_1_buffer[i]
							hdop_result = hdop_result + hdop_1_buffer[i]
							gpa_result = gpa_result + gpa_1_buffer[i]
						end	
						nsats_result = nsats_result / count
						hdop_result = hdop_result / count
						gpa_result = gpa_result / count
						print_debug("NSats " .. id .. ": " .. nsats_result)
						print_debug("HDop " .. id .. ": " .. hdop_result)
						print_debug("GPA delta " .. id .. ": " .. gpa_result)
						if (nsats_result < min_nsats) then
							warning_to_gcs("NSats 1 < " .. min_nsats .. " (" .. nsats_result .. ")")
						end
						if (hdop_result > max_hdop) then
							warning_to_gcs("HDop 1 > " .. max_hdop .. " (" .. hdop_result .. ")")
						end
						if (gpa_result > max_gpa_delta) then
							warning_to_gcs("GPA delta 1 > " .. max_gpa_delta .. " (" .. gpa_result .. ")")
						end
					else
						warning_to_gcs("GPS-1 buffer size: " .. count)
					end
				end
				if (id == 2) then
					local nsats_result = 0
					local hdop_result = 0
					local gpa_result = 0
					local count = sizeof(nsats_2_buffer)
					if (count == 10) then
						for i = 1, 10 do
							nsats_result = nsats_result + nsats_2_buffer[i]
							hdop_result = hdop_result + hdop_2_buffer[i]
							gpa_result = gpa_result + gpa_2_buffer[i]
						end	
						nsats_result = nsats_result / count
						hdop_result = hdop_result / count
						gpa_result = gpa_result / count
						print_debug("NSats " .. id .. ": " .. nsats_result)
						print_debug("HDop " .. id .. ": " .. hdop_result)
						print_debug("GPA delta " .. id .. ": " .. gpa_result)
						if (nsats_result < min_nsats) then
							warning_to_gcs("NSats 2 < " .. min_nsats .. " (" .. nsats_result .. ")")
						end
						if (hdop_result > max_hdop) then
							warning_to_gcs("HDop 2 > " .. max_hdop .. " (" .. hdop_result .. ")")
						end
						if (gpa_result > max_gpa_delta) then
							warning_to_gcs("GPA delta 2 > " .. max_gpa_delta .. " (" .. gpa_result .. ")")
						end
					else
						warning_to_gcs("GPS-2 buffer size: " .. count)
					end
				end
			end
		end
	else
		warning_to_gcs("gps was nil...")
	end
end


delta_posd_buffer = { }
delta_posd_end = 1
delta_posd_len = 10


function put_delta_posd()
	if not (ahrs == nil) then
		local n_ekf3_cores = ahrs:get_num_ekf3_cores()
		if (n_ekf3_cores == 2) then
			delta_posd_buffer[delta_posd_end] = ahrs:get_delta_posd()
			delta_posd_end = delta_posd_end + 1
			if (delta_posd_end > delta_posd_len) then
				delta_posd_end = 1
			end
		end
	else
		warning_to_gcs("ahrs was nil...")
	end
end


function check_average_delta_posd()
	result = 0
	count = sizeof(delta_posd_buffer)
	if (count == 0) then
		return
	end
	for i = 1, count do
		result = result + delta_posd_buffer[i]
	end
	result = result / count
	print_debug("Delta posD: " .. result)
	if (result > max_delta_posd) then
		warning_to_gcs("Delta posD > " .. max_delta_posd .. " (" .. result .. ")")
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
					print_debug("Vel. variance " .. id .. ": " .. vel_result)
					print_debug("Pos. variance " .. id .. ": " .. pos_result)
					print_debug("Hgt. variance " .. id .. ": " .. hgt_result)
					print_debug("Mag. variance " .. id .. ": " .. mag_result)
					print_debug("Tas. variance " .. id .. ": " .. tas_result)
					if (vel_result > max_vel_var) then
						warning_to_gcs("Vel. variance " .. id .. " > " .. max_vel_var .. " (" .. vel_result .. ")")
					end
					if (pos_result > max_pos_var) then
						warning_to_gcs("Pos. variance " .. id .. " > " .. max_pos_var .. " (" .. pos_result .. ")")
					end
					if (hgt_result >max_hgt_var) then
						warning_to_gcs("Hgt. variance " .. id .. " > " .. max_hgt_var .. " (" .. hgt_result .. ")")
					end
					if (mag_result > max_mag_var) then
						warning_to_gcs("Mag. variance " .. id .. " > " .. max_mag_var .. " (" .. mag_result .. ")")
					end
					if (tas_result > max_tas_var) then
						warning_to_gcs("Tas. variance " .. id .. " > " .. max_tas_var .. " (" .. tas_result .. ")")
					end
				else
					warning_to_gcs("XKF4 buffer size: " .. count)
				end
			end
		end
	else
		warning_to_gcs("ahrs was nil...")
	end
end


lat_prev = 0.0
lng_prev = 0.0
function check_speed()
	if not (ahrs == nil) then
		local current_pos = ahrs:get_position()
		if current_pos then
			local lat_curr = wrap_360_cd(current_pos:lat() * 0.0000001)
			local lng_curr = wrap_360_cd(current_pos:lng() * 0.0000001)
			local delta_lat = lat_curr - lat_prev
			local delta_lng = lng_curr - lng_prev
			local ds = 0.0
			if (math.abs(delta_lat) > 0.00000001) and (math.abs(delta_lng) > 0.00000001) then
				if (math.abs(delta_lat) < 0.01) or (math.abs(delta_lng) < 0.01) then
					--
					-- This scaling is necessary because the floating point
					-- precision seems to be insufficient to do trigonometry
					-- with such tiny angles. This is an ugly hack, there must
					-- be a more elegant solution
					--
					local delta_lat_scaled = delta_lat * 1000.0
					local delta_lng_scaled = delta_lat * 1000.0
					local lat_curr_scaled = lat_prev + delta_lat_scaled
					local lng_curr_scaled = lng_prev + delta_lng_scaled
					ds = dist(wrap_360_cd(lat_prev), wrap_360_cd(lng_prev), wrap_360_cd(lat_curr_scaled), wrap_360_cd(lng_curr_scaled)) * 0.001
				else
					ds = dist(wrap_360_cd(lat_prev), wrap_360_cd(lng_prev), wrap_360_cd(lat_curr), wrap_360_cd(lng_curr))
				end
			end
			print_debug("lat_prev: " .. lat_prev .. " | lng_prev: " .. lng_prev)
			print_debug("lat_curr: " .. lat_curr .. " | lng_curr: " .. lng_curr)
			print_debug("ds: " .. ds)
			if (ds > max_speed) then
				warning_to_gcs("Calculated speed > " .. max_speed .. "m/s (" .. ds .. "m/s)")
			end
			lat_prev = lat_curr
			lng_prev = lng_curr
		end
	else
		warning_to_gcs("ahrs was nil...")
	end
end


function test_dist()
	local test_lat_1 = 0.0
	local test_lng_1 = 0.0
	local test_lat_2 = 0.002
	local test_lng_2 = 0.002
	
	local delta_lat = test_lat_2 - test_lat_1
	local delta_lng = test_lng_2 - test_lng_1
	
	local d = 0.0
	
	if (math.abs(delta_lat) < 0.01) or (math.abs(delta_lng) < 0.01) then
		local delta_lat_scaled = delta_lat * 1000.0
		local delta_lng_scaled = delta_lat * 1000.0
		test_lat_2 = test_lat_1 + delta_lat_scaled
		test_lng_2 = test_lng_1 + delta_lng_scaled
		d = dist(wrap_360_cd(test_lat_1), wrap_360_cd(test_lng_1), wrap_360_cd(test_lat_2), wrap_360_cd(test_lng_2)) * 0.001
	else
		d = dist(wrap_360_cd(test_lat_1), wrap_360_cd(test_lng_1), wrap_360_cd(test_lat_2), wrap_360_cd(test_lng_2))
	end

	warning_to_gcs("Distance: " .. d .. "m")
end


function run_50Hz_loop()
	--debug_output = debug_output .. "-"
end


function run_20Hz_loop()
	--debug_output = debug_output .. "^"
end


function run_10Hz_loop()
	debug_output = debug_output .. "*"
	put_voltage()
	put_vibe()
	put_attitude()
	put_gps()
	put_delta_posd()
	put_xkf4()
end


function run_5Hz_loop()
	debug_output = debug_output .. "+"
end


function run_2Hz_loop()
	debug_output = debug_output .. "o"
	--check_attitude()
end


function run_1Hz_loop()
	debug_output = debug_output .. "O"
	check_average_voltage()
	check_average_vibe()
	check_average_attitude()
	check_average_gps()
	check_average_delta_posd()
	check_average_exkf4()
	check_speed()
	--test_dist()
end


cycle_count = 0
function update()

	if (math.fmod(cycle_count, 2) == 0) then
		run_50Hz_loop()
	end

	if (math.fmod(cycle_count, 5) == 0) then
		run_20Hz_loop()
	end

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
