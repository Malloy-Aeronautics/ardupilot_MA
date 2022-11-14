tick_interval = 10
debug_mode = true


function print_debug(msg)
	if (debug_mode == true) then
		gcs:send_text(6, "[MA_Script] " .. msg)
	end
end


text_output = ""


function run_50Hz_loop()
	--print_debug("50Hz")
	text_output = text_output .. "-"
end


function run_20Hz_loop()
	--print_debug("20Hz")
	text_output = text_output .. "^"
end


function run_10Hz_loop()
	--print_debug("10Hz")
	text_output = text_output .. "*"
end


function run_5Hz_loop()
	--print_debug("5Hz")
	text_output = text_output .. "+"
end


function run_2Hz_loop()
	--print_debug("2Hz")
	text_output = text_output .. "o"
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
	
	cycle_count = cycle_count + 1

	if (cycle_count >= 100) then
		if (debug_mode == true) then
			gcs:send_text(6, text_output)
		end
		text_output = ""
		cycle_count = 0
	end

	return update, tick_interval

end


return update()
