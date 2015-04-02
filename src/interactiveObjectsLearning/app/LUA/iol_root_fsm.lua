
require("iol_interact_fsm")

return rfsm.state {

	----------------------------------
	-- state INITPORTS 	            --
	----------------------------------
	ST_INITPORTS = rfsm.state{
		entry=function()
			ret = ispeak_port:open("/IOL/speak")
			ret = ret and speechRecog_port:open("/IOL/speechRecog")
			ret = ret and iol_port:open("/IOL/iolmanager")

			if ret == false then
				rfsm.send_events(fsm, 'e_error')
			else
				rfsm.send_events(fsm, 'e_connect')
			end
		end
	},

	----------------------------------
	-- state CONNECTPORTS           --
	----------------------------------
	ST_CONNECTPORTS = rfsm.state{
		entry=function()
			ret = yarp.NetworkBase_connect(ispeak_port:getName():c_str(), "/iSpeak")
			ret =  ret and yarp.NetworkBase_connect(speechRecog_port:getName():c_str(), "/speechRecognizer/rpc")
			ret =  ret and yarp.NetworkBase_connect(iol_port:getName():c_str(), "/iolStateMachineHandler/human:rpc")
			if ret == false then
				print("\n\nERROR WITH CONNECTIONS, PLEASE CHECK\n\n")
				rfsm.send_events(fsm, 'e_error')
			end
		end
	},

	----------------------------------
	-- state INITVOCABS             --
	----------------------------------
	ST_INITVOCABS = rfsm.state{
		entry=function()
			ret = true
			for key, word in pairs(objects) do
				ret = ret and (SM_RGM_Expand(speechRecog_port, "#Object", word) == "OK")
			end

			SM_Expand_asyncrecog(speechRecog_port, "icub-stop-now")

			if ret == false then
				rfsm.send_events(fsm, 'e_error')
			end
		end
	},

    ----------------------------------
	-- state HOME                   --
	----------------------------------
	ST_HOME = rfsm.state{
		entry=function()
			print("everything is fine, going home!")
			speak(ispeak_port, "Ready")
			IOL_goHome(iol_port)
		end
	},


	----------------------------------
	-- state FATAL                  --
	----------------------------------
	ST_FATAL = rfsm.state{
		entry=function()
			print("Fatal!")
			shouldExit = true;
		end
	},

	----------------------------------
	-- state FINI                   --
	----------------------------------
	ST_FINI = rfsm.state{
		entry=function()
			print("Closing...")
			yarp.NetworkBase_disconnect(ispeak_port:getName():c_str(), "/iSpeak")
			yarp.NetworkBase_disconnect(speechRecog_port:getName():c_str(), "/speechRecognizer/rpc")
			yarp.NetworkBase_disconnect(iol_port:getName():c_str(), "/iolStateMachineHandler/human:rpc")
			ispeak_port:close()
			speechRecog_port:close()
			iol_port:close()

			shouldExit = true;
		end
	},


	--------------------------------------------
	-- state MENU  is defined in menu_fsm.lua --
	--------------------------------------------
	ST_INTERACT = interact_fsm,


	----------------------------------
	-- setting the transitions      --
	----------------------------------

	rfsm.transition { src='initial', tgt='ST_INITPORTS' },
	rfsm.transition { src='ST_INITPORTS', tgt='ST_CONNECTPORTS', events={ 'e_connect' } },
	rfsm.transition { src='ST_INITPORTS', tgt='ST_FATAL', events={ 'e_error' } },

    rfsm.transition { src='ST_CONNECTPORTS', tgt='ST_FINI', events={ 'e_error' } },
	rfsm.transition { src='ST_CONNECTPORTS', tgt='ST_INITVOCABS', events={ 'e_done' } },

	rfsm.transition { src='ST_INITVOCABS', tgt='ST_FINI', events={ 'e_error' } },
	rfsm.transition { src='ST_INITVOCABS', tgt='ST_HOME', events={ 'e_done' } },

	rfsm.transition { src='ST_HOME', tgt='ST_INTERACT', events={ 'e_done' } },
	rfsm.transition { src='ST_INTERACT', tgt='ST_FINI', events={ 'e_menu_done' } },

}
