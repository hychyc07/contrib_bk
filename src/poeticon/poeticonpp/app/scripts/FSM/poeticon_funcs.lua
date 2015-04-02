
function speak(port, str)
   local wb = port:prepare()
    wb:clear()
    wb:addString(str)
    port:write()
end

function SM_RGM_Expand(port, vocab, word)
    local wb = port:prepare()
    wb:clear()
    wb:addString("RGM")
	wb:addString("vocabulary")
	wb:addString("add")
	wb:addString(vocab)
	wb:addString(word)
    port:write(wb,reply)
	return reply:get(1):asString()
end


function SM_Expand_asyncrecog(port, gram)
    local wb = port:prepare()
    wb:clear()
    wb:addString("asyncrecog")
	wb:addString("addGrammar")
	wb:addString(gram)
    port:write(wb,reply)
end
