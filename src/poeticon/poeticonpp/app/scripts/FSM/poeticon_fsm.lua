
return rfsm.state {

	----------------------------------
	-- state INITPORTS 	            --
	----------------------------------
	ST_INITPORTS = rfsm.state{
		entry=function()
			ret = ispeak_port:open("/poeticon/speak")
			ret = ret and speechRecog_port:open("/poeticon/speechRecog")
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
			if ret == false then
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
				ret = ret and (SM_RGM_Expand(speechRecog_port, "#object", word) == "OK")
			end

			for key, word in pairs(actions) do
				ret = ret and (SM_RGM_Expand(speechRecog_port, "#action", word) == "OK")
			end

			SM_Expand_asyncrecog(speechRecog_port, "icub-stop-now")

			if ret == false then
				rfsm.send_events(fsm, 'e_error')
			end
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
			ispeak_port:close()
			speechRecog_port:close()
			shouldExit = true;
		end
	},


	----------------------------------
	-- setting the transitions      --
	----------------------------------

	rfsm.transition { src='initial', tgt='ST_INITPORTS' },
	rfsm.transition { src='ST_INITPORTS', tgt='ST_CONNECTpORTS', events={ 'e_connect' } },
	rfsm.transition { src='ST_INITPORTS', tgt='ST_FATAL', events={ 'e_error' } },

    rfsm.transition { src='ST_CONNECTpORTS', tgt='ST_FINI', events={ 'e_error' } },
	rfsm.transition { src='ST_CONNECTpORTS', tgt='ST_INITVOCABS', events={ 'e_done' } },

	rfsm.transition { src='ST_INITVOCABS', tgt='ST_FINI', events={ 'e_error' } },

  --[[
   ali = rfsm.state{
            entry=function() print("entered Ali") end,
            doo = function()
                while true do
                    print("staying with Ali")
                    rfsm.yield(true)
                end
             end,
            },
	--]]
}
