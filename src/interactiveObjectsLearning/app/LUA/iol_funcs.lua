
function speak(port, str)
   local wb = port:prepare()
    wb:clear()
    wb:addString(str)
    port:write()
	yarp.Time_delay(1.0)
end

----------------------------------
-- functions MOTOR - IOL        --
----------------------------------

function IOL_goHome(port)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString("home")
    port:write(wb,reply)
end

function IOL_calibrate(port)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString("cata")
    port:write(wb,reply)
end

function IOL_where_is(port, objName)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString("where")
	wb:addString(objName)
    port:write(wb,reply)
	return reply
end

function IOL_reward(port, reward)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString(reward)
    port:write(wb,reply)
end

function IOL_track_start(port)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString("track")
	wb:addString("start")
    port:write(wb,reply)
end

function IOL_track_stop(port)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString("track")
	wb:addString("stop")
    port:write(wb,reply)
end

function IOL_populate_name(port, objName)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString("name")
	wb:addString(objName)
    port:write(wb,reply)
	return reply:get(0):asString():c_str()
end

function IOL_take(port, objName)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString("take")
	wb:addString(objName)
    port:write(wb,reply)
	return reply:get(0):asString():c_str()
end

function IOL_touch(port, objName)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString("touch")
	wb:addString(objName)
    port:write(wb,reply)
	return reply:get(0):asString():c_str()
end

function IOL_push(port, objName)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString("push")
	wb:addString(objName)
    port:write(wb,reply)
	return reply:get(0):asString():c_str()
end

function IOL_forget(port, objName)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString("forget")
	wb:addString(objName)
    port:write(wb,reply)
	return reply:get(0):asString():c_str()
end

function IOL_explore(port, objName)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString("explore")
	wb:addString(objName)
    port:write(wb,reply)
	return reply:get(0):asString():c_str()
end

function IOL_what(port)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString("what")
    port:write(wb,reply)
	return reply:get(0):asString():c_str()
end

function IOL_calib_kin_start(port, side, objName)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString("caki")
	wb:addString("start")
	wb:addString(side)
	wb:addString(objName)
    port:write(wb,reply)
	return reply:get(0):asString():c_str()
end

function IOL_calib_kin_stop(port)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString("caki")
	wb:addString("stop")
    port:write(wb,reply)
	return reply:get(0):asString():c_str()
end

----------------------------------
-- functions SPEECH             --
----------------------------------

function SM_RGM_Expand(port, vocab, word)
    local wb = yarp.Bottle()
	local reply = yarp.Bottle()
    wb:clear()
    wb:addString("RGM")
	wb:addString("vocabulory")
	wb:addString("add")
	wb:addString(vocab)
	wb:addString(word)
    port:write(wb,reply)
	--print(reply:get(1):asString():c_str())
	return reply:get(1):asString():c_str()
end

function SM_Expand_asyncrecog(port, gram)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString("asyncrecog")
	wb:addString("addGrammar")
	wb:addString(gram)
    port:write(wb,reply)
end

function SM_Reco_Grammar(port, gram)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString("recog")
	wb:addString("grammarSimple")
	wb:addString(gram)
    port:write(wb,reply)
	return reply
end

function SM_RGM_Expand_Auto(port, vocab)
	local wb = yarp.Bottle()
	local reply = yarp.Bottle()
	wb:clear()
    wb:addString("RGM")
	wb:addString("vocabulory")
	wb:addString("addAuto")
	wb:addString(vocab)
    port:write(wb,reply)
	return reply:get(1):asString():c_str()
end

--[[
proc SM_Reco_Grammar { gram } {

	bottle clear
	bottle addString "recog"
	bottle addString "grammarSimple"
	bottle addString $gram
	SpeechManagerPort write bottle reply
	puts "Received from SpeechManager : [reply toString] "
	set wordsList ""
	for { set i 1 } { $i< [reply size] } {incr i 2} {
		set wordsList [lappend wordsList [ [reply get $i] toString] ]
	}
	return $wordsList
}
]]--
