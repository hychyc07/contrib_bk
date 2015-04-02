
event_table = {
	See			= "e_exit",
	Calibrate   = "e_calibrate",
	Where	    = "e_where",
	I	    	= "e_i_will",
	Take    	= "e_take",
	Return    	= "e_home",
	Touch    	= "e_touch",
	Push    	= "e_push",
	Forget    	= "e_forget",
	Explore    	= "e_explore",
	What    	= "e_what",
	Let    		= "e_let",
	}

interact_fsm = rfsm.state{

	----------------------------------
	-- state SUB_MENU               --
	----------------------------------
	SUB_MENU = rfsm.state{
		entry=function()
			print("in substate MENU : waiting for speech command!")
		end,

		doo = function()
            while true do
				speak(ispeak_port, "What should I do?")
				result = SM_Reco_Grammar(speechRecog_port, grammar)
				print("received REPLY: ", result:toString() )
				local cmd =  result:get(1):asString():c_str()
				rfsm.send_events(fsm, event_table[cmd])
                rfsm.yield(true)
            end
		end
	},

	----------------------------------
	-- states                       --
	----------------------------------

	SUB_EXIT = rfsm.state{
		entry=function()
			speak(ispeak_port, "Ok, bye bye")
			rfsm.send_events(fsm, 'e_menu_done')
		end
	},

	SUB_CALIBRATE = rfsm.state{
		entry=function()
			IOL_calibrate(iol_port)
			speak(ispeak_port,"OK, I know the table height")
		end
	},

	SUB_WHERE = rfsm.state{
		entry=function()
			local obj = result:get(7):asString():c_str()

			local b = IOL_where_is(iol_port, obj)
			local ret = b:get(0):asString():c_str()
			if  ret == "ack" or ret == "nack" then
				local reward = SM_Reco_Grammar(speechRecog_port, grammar_reward)
				print("received REPLY: ", reward:toString() )
				local cmd  =  reward:get(1):asString():c_str()
				print("REWARD IS", cmd)
				if cmd == "Yes" then
					IOL_reward(iol_port,"ack")
				elseif cmd == "No" then
					IOL_reward(iol_port,"nack")
				elseif cmd == "Skip" then
					IOL_reward(iol_port,"skip")
				else
					IOL_reward(iol_port,"skip")
					speak(ispeak_port,"I don't understand")
				end

			else
				print("I dont get it")
				speak(ispeak_port,"something is wrong")
			end
		end
	},

	SUB_TEACH_OBJ = rfsm.state{
		entry=function()
			IOL_track_start(iol_port)
			local answer = SM_Reco_Grammar(speechRecog_port, grammar_track)
			print("received REPLY: ", answer:toString() )
			local cmd  =  answer:get(1):asString():c_str()
			print("REPLY IS", cmd)
			if cmd == "There" then
				print("in there")
				IOL_track_stop(iol_port)
				print("stopped track")
				local str = SM_RGM_Expand_Auto(speechRecog_port, "#Object")
				print("done with name ", str)
				local ret = IOL_populate_name(iol_port, str)
				print("REPLY IS", ret)

			elseif cmd == "Skip" then
				IOL_track_stop(iol_port)
			else
				IOL_track_stop(iol_port)
				speak(ispeak_port,"I don't understand")
			end
		end
	},

	SUB_TAKE = rfsm.state{
		entry=function()
			local obj = result:get(5):asString():c_str()
			local b = IOL_take(iol_port, obj)
		end
	},

	SUB_RETURN = rfsm.state{
		entry=function()
			print("going home!")
			IOL_goHome(iol_port)
		end
	},

	SUB_TOUCH = rfsm.state{
		entry=function()
			local obj = result:get(5):asString():c_str()
			local b = IOL_touch(iol_port, obj)
		end
	},

	SUB_PUSH = rfsm.state{
		entry=function()
			local obj = result:get(5):asString():c_str()
			local b = IOL_push(iol_port, obj)
		end
	},

	SUB_FORGET = rfsm.state{
		entry=function()
			local obj = result:get(3):asString():c_str()
			local b = IOL_forget(iol_port, obj)
		end
	},

	SUB_EXPLORE = rfsm.state{
		entry=function()
			local obj = result:get(5):asString():c_str()
			local b = IOL_explore(iol_port, obj)
		end
	},

	SUB_WHAT = rfsm.state{
		entry=function()
			--
			local answer = IOL_what(iol_port)
			if  answer == "ack" then
				local reward = SM_Reco_Grammar(speechRecog_port, grammar_whatAck)
				print("received REPLY: ", reward:toString() )
				local cmd  =  reward:get(1):asString():c_str()
				local obj  =  reward:get(9):asString():c_str() --------TEST !!!!!!!
				print("REWARD IS", cmd)
				if cmd == "Yes" then
					IOL_reward(iol_port,"ack")
				elseif cmd == "No" then
					IOL_reward(iol_port,"nack")
				elseif cmd == "Skip" then
					IOL_reward(iol_port,"skip")
				elseif cmd == "Wrong" then
					local ret = IOL_populate_name(iol_port, obj)
					print("REPLY IS", ret)
				else
					IOL_reward(iol_port,"skip")
					speak(ispeak_port,"I don't understand")
				end
			elseif answer == "nack" then
				local reward = SM_Reco_Grammar(speechRecog_port, grammar_whatNack)
				print("received REPLY: ", reward:toString() )
				local cmd  =  reward:get(1):asString():c_str()
				local obj  =  reward:get(7):asString():c_str()
				if cmd == "Skip" then
					IOL_reward(iol_port,"skip")
				else
					local ret = IOL_populate_name(iol_port, obj)
					print("REPLY IS", ret)
				end
			else
				print("I dont get it")
				speak(ispeak_port,"something is wrong")
			end
		end
	},

	SUB_LET = rfsm.state{
		entry=function()
			let_obj = result:get(17):asString():c_str()
			let_arm = result:get(23):asString():c_str()
			speak(ispeak_port,"Do you mean show me how to reach the " .. let_obj .. " with my " .. let_arm .." arm? ")
			local ret  = SM_Reco_Grammar(speechRecog_port, grammar_teach)
			let_cmd  =  ret:get(1):asString():c_str()
			end,

			doo = function()
				while let_cmd == "Yes" do
					print("Enter teaching mode")
					print("arm is ", let_arm)
					print("Obj is ", let_obj)
					local ret = IOL_calib_kin_start(iol_port, let_arm, let_obj)
					if  ret == "ack" then
						rfsm.send_events(fsm, "e_kin")
					else
						local fin  = SM_Reco_Grammar(speechRecog_port, grammar_teach)
						local cmd  =  fin:get(1):asString():c_str()
						if cmd == "No" then
							let_cmd = "done"
						elseif cmd ~= "Yes" then
							speak(ispeak_port, "Sorry I do not understand")
						end
					end
					rfsm.yield(true)
				end
			end
	},

	SUB_KIN = rfsm.state{
		doo = function()
			local finish = false
			while not finish do
				local fin  = SM_Reco_Grammar(speechRecog_port, grammar_teach)
				local cmd  =  fin:get(1):asString():c_str()
				if cmd == "Finished" then
					IOL_calib_kin_stop(iol_port)
					finish = true
				end
				rfsm.yield(true)
			end
		end
	},

	----------------------------------
	-- state transitions            --
	----------------------------------

	rfsm.trans{ src='initial', tgt='SUB_MENU'},
	rfsm.transition { src='SUB_MENU', tgt='SUB_EXIT', events={ 'e_exit' } },

	rfsm.transition { src='SUB_MENU', tgt='SUB_CALIBRATE', events={ 'e_calibrate' } },
	rfsm.transition { src='SUB_CALIBRATE', tgt='SUB_MENU', events={ 'e_done' } },

	rfsm.transition { src='SUB_MENU', tgt='SUB_WHERE', events={ 'e_where' } },
	rfsm.transition { src='SUB_WHERE', tgt='SUB_MENU', events={ 'e_done' } },

	rfsm.transition { src='SUB_MENU', tgt='SUB_TEACH_OBJ', events={ 'e_i_will' } },
	rfsm.transition { src='SUB_TEACH_OBJ', tgt='SUB_MENU', events={ 'e_done' } },

	rfsm.transition { src='SUB_MENU', tgt='SUB_TAKE', events={ 'e_take' } },
	rfsm.transition { src='SUB_TAKE', tgt='SUB_MENU', events={ 'e_done' } },

	rfsm.transition { src='SUB_MENU', tgt='SUB_RETURN', events={ 'e_home' } },
	rfsm.transition { src='SUB_RETURN', tgt='SUB_MENU', events={ 'e_done' } },

	rfsm.transition { src='SUB_MENU', tgt='SUB_TOUCH', events={ 'e_touch' } },
	rfsm.transition { src='SUB_TOUCH', tgt='SUB_MENU', events={ 'e_done' } },

	rfsm.transition { src='SUB_MENU', tgt='SUB_PUSH', events={ 'e_push' } },
	rfsm.transition { src='SUB_PUSH', tgt='SUB_MENU', events={ 'e_done' } },

	rfsm.transition { src='SUB_MENU', tgt='SUB_FORGET', events={ 'e_forget' } },
	rfsm.transition { src='SUB_FORGET', tgt='SUB_MENU', events={ 'e_done' } },

	rfsm.transition { src='SUB_MENU', tgt='SUB_EXPLORE', events={ 'e_explore' } },
	rfsm.transition { src='SUB_EXPLORE', tgt='SUB_MENU', events={ 'e_done' } },

	rfsm.transition { src='SUB_MENU', tgt='SUB_WHAT', events={ 'e_what' } },
	rfsm.transition { src='SUB_WHAT', tgt='SUB_MENU', events={ 'e_done' } },

	rfsm.transition { src='SUB_MENU', tgt='SUB_LET', events={ 'e_let' } },
	rfsm.transition { src='SUB_LET', tgt='SUB_MENU', events={ 'e_done' } },

	rfsm.transition { src='SUB_LET', tgt='SUB_KIN', events={ 'e_kin' } },
	rfsm.transition { src='SUB_KIN', tgt='SUB_MENU', events={ 'e_done' } },

}
