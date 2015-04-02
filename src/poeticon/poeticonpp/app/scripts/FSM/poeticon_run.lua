#!/usr/bin/lua

require("rfsm")
require("yarp")

require("poeticon_funcs")

yarp.Network()

-------
shouldExit = false

-- initilization
ispeak_port = yarp.BufferedPortBottle()
speechRecog_port = yarp.Port()

-- defining objects and actions vocabularies
objects = {"octopus", "lego", "toy", "ladybug", "turtle", "cat"}
actions = {"{point at}", "{what is this}"}

-- load state machine model and initalize it
fsm_model = rfsm.load("./poeticon_fsm.lua")
fsm = rfsm.init(fsm_model)
rfsm.run(fsm)


repeat
	print("waiting for commands...")
    rfsm.run(fsm)
    yarp.Time_delay(0.1)
until shouldExit ~= false

print
-- Deinitialize yarp network
yarp.Network_fini()





--[[
--receiver = yarp.BufferedPortBottle()
--receiver:open("/rfsm/get")

    -- read from receiver port
    local rb = receiver:read(true);
    if rb ~= nil then
        print("Received: ", rb:toString())
        rfsm.send_events(fsm, rb:toString())
    end
	--]]
