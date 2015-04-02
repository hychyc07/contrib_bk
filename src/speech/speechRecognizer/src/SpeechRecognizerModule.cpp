#include "SpeechRecognizerModule.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Helpers for dealing with the weird strings of windows... 
std::wstring s2ws(const std::string& s)
{
    int len;
    int slength = (int)s.length() + 1;
    len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0); 
    wchar_t* buf = new wchar_t[len];
    MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
    std::wstring r(buf);
    delete[] buf;
    return r;
}
std::string ws2s(LPCWSTR s)
{
   char    *pmbbuf   = (char *)malloc( 100 );
   wcstombs( pmbbuf, s, 100 );
   return pmbbuf;
}
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}
std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Module implementation
bool SpeechRecognizerModule::configure(ResourceFinder &rf )
{    
    setName( rf.check("name",Value("speechRecognizer")).asString().c_str() );
    m_timeout = rf.check("timeout",Value(10000)).asInt();

    //Deal with speech recognition
    string grammarFile = rf.findFile( rf.check("grammarFile",Value("defaultGrammar.grxml")).asString().c_str() );
    std::wstring tmp = s2ws(grammarFile);
    LPCWSTR cwgrammarfile = tmp.c_str();

    m_useTalkBack = rf.check("talkback");

    //Initialise the speech crap
    bool everythingIsFine = true;
    HRESULT hr;
    everythingIsFine &= SUCCEEDED( m_cpRecoEngine.CoCreateInstance(CLSID_SpInprocRecognizer));
    everythingIsFine &= SUCCEEDED( SpCreateDefaultObjectFromCategoryId(SPCAT_AUDIOIN, &m_cpAudio));
    everythingIsFine &= SUCCEEDED( m_cpRecoEngine->SetInput(m_cpAudio, TRUE));
    everythingIsFine &= SUCCEEDED( m_cpRecoEngine->CreateRecoContext( &m_cpRecoCtxt ));

    everythingIsFine &=  SUCCEEDED(hr = m_cpRecoCtxt->SetNotifyWin32Event()) ;
    everythingIsFine &= SUCCEEDED(hr = m_cpRecoCtxt->SetInterest(SPFEI(SPEI_RECOGNITION), SPFEI(SPEI_RECOGNITION))) ;

    //Load grammar from file
    everythingIsFine &= SUCCEEDED( m_cpRecoCtxt->CreateGrammar( 1, &m_cpGrammarFromFile ));
    everythingIsFine &= SUCCEEDED( m_cpGrammarFromFile->SetGrammarState(SPGS_DISABLED));
    everythingIsFine &= SUCCEEDED( m_cpGrammarFromFile->LoadCmdFromFile(cwgrammarfile, SPLO_DYNAMIC));

    //Create a runtime grammar
    everythingIsFine &= SUCCEEDED( m_cpRecoCtxt->CreateGrammar( 2, &m_cpGrammarRuntime ));
    everythingIsFine &= SUCCEEDED( m_cpGrammarRuntime->SetGrammarState(SPGS_DISABLED));
        
    //Create a dictation grammar
    everythingIsFine &= SUCCEEDED( m_cpRecoCtxt->CreateGrammar( 0, &m_cpGrammarDictation ));
    everythingIsFine &= SUCCEEDED( m_cpGrammarDictation->SetGrammarState(SPGS_DISABLED));
    everythingIsFine &= SUCCEEDED(hr =m_cpGrammarDictation->LoadDictation(NULL, SPLO_STATIC));
    everythingIsFine &= SUCCEEDED(m_cpGrammarDictation->SetDictationState(SPRS_INACTIVE));
    
    if( everythingIsFine )
    {
        string pName = "/";
        pName += getName();
        pName += "/recog/continuous:o";
        m_portContinuousRecognition.open( pName.c_str() );

        pName = "/";
        pName += getName();
        pName += "/tts/iSpeak:o";
        m_port2iSpeak.open( pName.c_str() );
        if (Network::connect(m_port2iSpeak.getName().c_str(),"/iSpeak"))
        {
            cout<<"Connection to iSpeak succesfull"<<endl;
            //say("Speech recognizer is running");
        }
        else
            cout<<"Unable to connect to iSpeak. Connect manually."<<endl;

        pName = "/";
        pName += getName();
        pName += "/rpc";
        m_portRPC.open( pName.c_str() );
        attach(m_portRPC);

        //Start recognition
        everythingIsFine &= SUCCEEDED(m_cpRecoEngine->SetRecoState(SPRST_ACTIVE_ALWAYS));
        everythingIsFine &= SUCCEEDED(m_cpRecoCtxt->SetInterest(SPFEI(SPEI_RECOGNITION), SPFEI(SPEI_RECOGNITION)));
        everythingIsFine &= SUCCEEDED(m_cpGrammarFromFile->SetRuleState(NULL, NULL, SPRS_ACTIVE));
        everythingIsFine &= SUCCEEDED( m_cpGrammarFromFile->SetGrammarState(SPGS_ENABLED));

        //setRuntimeGrammar_Custom("take the toy|take the banana");
    }
    m_is_in_syncrecog = false;
    return (everythingIsFine);
}

/************************************************************************/
bool SpeechRecognizerModule::updateModule()
{
    if (!m_is_in_syncrecog)
    {
        //std::cout<<".";
        const float ConfidenceThreshold = 0.3f;
        SPEVENT curEvent;
        ULONG fetched = 0;
        HRESULT hr = S_OK;

        m_cpRecoCtxt->GetEvents(1, &curEvent, &fetched);

        while (fetched > 0)
        {
            ISpRecoResult* result = reinterpret_cast<ISpRecoResult*>(curEvent.lParam);
            SPPHRASE* pPhrase = NULL;

            //Do some smarter stuff with the recognition
            hr = result->GetPhrase(&pPhrase);
            if (SUCCEEDED(hr))
            {               
                //Sent the recognition to yarp
                CSpDynamicString dstrText;
                result->GetText(SP_GETWHOLEPHRASE, SP_GETWHOLEPHRASE, TRUE, &dstrText, NULL);
                string fullSentence = ws2s(dstrText);
            
                vector<string> words = split(fullSentence,' ');
                Bottle bOut;
                for(int w=0;w<words.size();w++)
                {
                    //Todo extract the confidence value somehow...
                    bOut.addString(words[w].c_str());
                    bOut.addDouble(1.0);
                }

                m_portContinuousRecognition.write(bOut);

                if (m_useTalkBack)
                    say(fullSentence);              
                //int wordCount = pPhrase->cbSize;
                //string rawPhrase = "";
                //for(int i=0; i< wordCount; i++)
                //    rawPhrase += ws2s(pPhrase->pElements[i].pszDisplayText) + " ";
                //std::cout<<"Raw sentence: "<<rawPhrase<<std::endl;

                if ((pPhrase->pProperties != NULL) && (pPhrase->pProperties->pFirstChild != NULL))
                {
                    const SPPHRASEPROPERTY* pSemanticTag = pPhrase->pProperties->pFirstChild;
                    if (pSemanticTag->SREngineConfidence > ConfidenceThreshold)
                    {
                        //std::cout<<pSemanticTag->pszValue<<std::endl;
                    }
                }
                ::CoTaskMemFree(pPhrase);
            }
            m_cpRecoCtxt->GetEvents(1, &curEvent, &fetched);
        }
    }
    return true;
}

/************************************************************************/
bool SpeechRecognizerModule::respond(const Bottle& cmd, Bottle& reply) 
{

    string firstVocab = cmd.get(0).asString().c_str();
    reply.addString("ACK");
    if (firstVocab == "quit")
    {
        return false;
    }
    if (firstVocab == "tts")
    {
        string sentence = cmd.get(1).asString().c_str();
        say(sentence);
        reply.addString("OK");
    }
    else if (firstVocab == "RGM" || firstVocab == "rgm" )
    {    
        string secondVocab = cmd.get(1).asString().c_str();
        if (secondVocab=="vocabulory")
            handleRGMCmd(cmd.tail().tail(), reply);
    }
    else if (firstVocab == "recog")
    {
        handleRecognitionCmd(cmd.tail(), reply);
    }
    else if (firstVocab == "asyncrecog")
    {
        handleAsyncRecognitionCmd(cmd.tail(), reply);
    }
    else
        reply.addString("UNKNOWN");

    return true;
}


/************************************************************************/
bool SpeechRecognizerModule::handleRGMCmd(const Bottle& cmd, Bottle& reply)
{
    string firstVocab = cmd.get(0).asString().c_str();
    if (firstVocab == "add")
    {
        string vocabulory = cmd.get(1).asString().c_str();
        if (vocabulory[0] != '#')
        {
            reply.addString("ERROR");
            //reply.addString("Vocabulories have to start with a #. #Dictation and #WildCard are reserved. Aborting.");
            return false;
        }
        string word = cmd.get(2).asString().c_str();
        m_vocabulories[vocabulory].push_back(word);
        reply.addString("OK");
        return true;
    }

    if (firstVocab == "addAuto")
    {
        string vocabulory = cmd.get(1).asString().c_str();
        string word = learnNewWord(vocabulory);
        m_vocabulories[vocabulory].push_back(word);
        reply.addString(word.c_str());
        return true;
    }
    reply.addString("UNKNOWN");
    return false;
}

/************************************************************************/
list< pair<string, double> > SpeechRecognizerModule::waitNextRecognitionDictationSpelling(int timeout)
{
    SUCCEEDED(m_cpGrammarDictation->UnloadDictation());
    SUCCEEDED(m_cpGrammarDictation->LoadDictation(L"spelling", SPLO_STATIC));
    SUCCEEDED(m_cpGrammarFromFile->SetGrammarState(SPGS_DISABLED)); 
    SUCCEEDED(m_cpGrammarRuntime->SetGrammarState(SPGS_DISABLED)); 
    SUCCEEDED(m_cpGrammarDictation->SetGrammarState(SPGS_ENABLED)); 
    SUCCEEDED(m_cpGrammarDictation->SetDictationState(SPRS_ACTIVE)); 
    
    list< pair<string, double> > results = waitNextRecognition(timeout);

    SUCCEEDED(m_cpGrammarDictation->UnloadDictation());
    SUCCEEDED(m_cpGrammarDictation->LoadDictation(NULL, SPLO_STATIC));
    SUCCEEDED(m_cpGrammarFromFile->SetGrammarState(SPGS_ENABLED)); 
    SUCCEEDED(m_cpGrammarRuntime->SetGrammarState(SPGS_DISABLED)); 
    SUCCEEDED(m_cpGrammarDictation->SetGrammarState(SPGS_DISABLED)); 
    SUCCEEDED(m_cpGrammarDictation->SetDictationState(SPRS_INACTIVE)); 

    return results;
}

/************************************************************************/
list< pair<string, double> > SpeechRecognizerModule::waitNextRecognitionDictation(int timeout)
{
    HRESULT hr; 
    bool everythingIsFine = SUCCEEDED(hr = m_cpGrammarFromFile->SetGrammarState(SPGS_DISABLED)); 
    everythingIsFine &= SUCCEEDED(hr = m_cpGrammarRuntime->SetGrammarState(SPGS_DISABLED)); 
    everythingIsFine &= SUCCEEDED(hr = m_cpGrammarDictation->SetGrammarState(SPGS_ENABLED)); 
    everythingIsFine &= SUCCEEDED(hr = m_cpGrammarDictation->SetDictationState(SPRS_ACTIVE)); 

    list< pair<string, double> > results = waitNextRecognition(timeout);

    everythingIsFine &= SUCCEEDED(hr = m_cpGrammarFromFile->SetGrammarState(SPGS_ENABLED)); 
    everythingIsFine &= SUCCEEDED(hr = m_cpGrammarRuntime->SetGrammarState(SPGS_DISABLED)); 
    everythingIsFine &= SUCCEEDED(hr = m_cpGrammarDictation->SetGrammarState(SPGS_DISABLED)); 
    everythingIsFine &= SUCCEEDED(hr = m_cpGrammarDictation->SetDictationState(SPRS_INACTIVE)); 
    return results;
}

/************************************************************************/
string SpeechRecognizerModule::learnNewWord(string vocabType)
{
    say("I will learn a new " + vocabType + ".");
    string word = "";
    bool wordIsOk = false;
    int trials = 0;
    while (!wordIsOk)
    {
        word = "";
        list< pair<string, double> > results;
        if (trials >= TRIAL_BEFORE_SPELLING)
        {
            //Spelling
            say("Could you spell the new word for me?");
            results = waitNextRecognitionDictationSpelling(m_timeout);


            if (results.size() > 0)
            {
                word = "";
                for(list< pair<string, double> >::iterator it = results.begin();it!=results.end();it++)
                {
                    word += it->first;
                }
            }
        }
        else
        {
            say("Please tell me the word.");
            results = waitNextRecognitionDictation(m_timeout);
            if (results.size() > 0)
                word = results.front().first;
        }
        trials++;
        if (word != "")
        {
            
            list< pair<string, double> > confirm;
            while (confirm.size()==0)
            {
                say("Did you say " + word + "?");
                setGrammarCustom(m_cpGrammarRuntime,"Yes I did | No I did not", false);
                confirm = waitNextRecognition(m_timeout);
            }

            if (confirm.front().first == "Yes")
                wordIsOk = true;
        }
        else
        {
            say("I didn't understand anything.");
        }
    }
    say("Thanks, now I can use the word " + word + ".");

    return word;
}
    
/************************************************************************/
bool SpeechRecognizerModule::handleAsyncRecognitionCmd(const Bottle& cmd, Bottle& reply)
{
    string firstVocab = cmd.get(0).asString().c_str();
    if (firstVocab == "clear")
    {
        bool everythingIsFine=true;
        HRESULT hr;
        SPSTATEHANDLE rootRule;
        m_cpGrammarFromFile->ResetGrammar(SpGetUserDefaultUILanguage());
        //everythingIsFine &= SUCCEEDED(hr = m_cpRecoCtxt->CreateGrammar( 1, &m_cpGrammarFromFile ));
        //everythingIsFine &= SUCCEEDED(hr = m_cpGrammarFromFile->GetRule(L"rootRule", NULL, SPRAF_TopLevel | SPRAF_Active, TRUE, &rootRule));
        //everythingIsFine &= SUCCEEDED(hr = m_cpGrammarFromFile->ClearRule(rootRule));
        everythingIsFine &= SUCCEEDED(hr = m_cpGrammarFromFile->Commit(NULL));
        everythingIsFine &= SUCCEEDED(hr = m_cpGrammarFromFile->SetGrammarState(SPGS_ENABLED));        
        everythingIsFine &= SUCCEEDED(hr = m_cpGrammarFromFile->SetRuleState(NULL, NULL, SPRS_ACTIVE));
        everythingIsFine &= SUCCEEDED(hr = m_cpRecoCtxt->Resume(0));
        reply.addString("Cleared");
        return true;
    }

    if (firstVocab == "addGrammar")
    {    
        string grammar = cmd.get(1).asString().c_str();
        bool everythingIsFine = setGrammarCustom(m_cpGrammarFromFile,grammar,true);
        reply.addString("Added");
        return true;
    }
    return false;
}


/************************************************************************/
bool SpeechRecognizerModule::handleRecognitionCmd(const Bottle& cmd, Bottle& reply)
{
    string firstVocab = cmd.get(0).asString().c_str();
    
    if (firstVocab == "timeout")
    {
        m_timeout = cmd.get(1).asInt();
        return true;
    }
    else if (firstVocab == "choices")
    {
        string choices ="";
        for (int wI = 1; wI < cmd.size(); wI++)
        {
            choices+=cmd.get(wI).asString().c_str();
            if (wI<cmd.size()-1)
                choices+="|";
        }
        setGrammarCustom(m_cpGrammarRuntime,choices,false);
        SUCCEEDED(m_cpGrammarFromFile->SetGrammarState(SPGS_DISABLED)); 
        SUCCEEDED(m_cpGrammarRuntime->SetGrammarState(SPGS_ENABLED)); 
    }
    else if (firstVocab == "dictation")
    {
        //Disable the from file grammar
        SUCCEEDED(m_cpGrammarFromFile->SetGrammarState(SPGS_DISABLED)); 
        SUCCEEDED(m_cpGrammarRuntime->SetGrammarState(SPGS_DISABLED)); 
        SUCCEEDED(m_cpGrammarDictation->SetGrammarState(SPGS_ENABLED)); 
        SUCCEEDED(m_cpGrammarDictation->SetDictationState(SPRS_ACTIVE));    
    }    
    else if (firstVocab == "grammarXML")
    {
        //todo
        cout<<"grammarXML not implemented yet."<<endl;
        reply.addString("-1");
        return false;
    }
    else if (firstVocab == "grammarSimple")
    {
        string RADStyle = cmd.get(1).asString().c_str();
        cout<<"Setting runtime grammar to : "<<RADStyle<<endl;
        setGrammarCustom(m_cpGrammarRuntime,RADStyle,false);
        SUCCEEDED(m_cpGrammarFromFile->SetGrammarState(SPGS_DISABLED)); 
        SUCCEEDED(m_cpGrammarRuntime->SetGrammarState(SPGS_ENABLED)); 
    }
    else 
    {
        reply.addString("UNKNWON");
        return false;
    }

    //Force blocking recognition
    list< pair<string, double> > results = waitNextRecognition(m_timeout);
    for(list< pair<string, double> >::iterator it = results.begin(); it != results.end(); it++)
    {
        reply.addString(it->first.c_str());
        reply.addDouble(it->second);
    }
    //Disable the runtime grammar
    SUCCEEDED(m_cpGrammarDictation->SetDictationState(SPRS_INACTIVE));  
    SUCCEEDED(m_cpGrammarRuntime->SetGrammarState(SPGS_DISABLED));  
    SUCCEEDED(m_cpGrammarDictation->SetGrammarState(SPGS_DISABLED));   
    SUCCEEDED(m_cpGrammarFromFile->SetGrammarState(SPGS_ENABLED));        
    return true;
}

/************************************************************************/
list< pair<string, double> > SpeechRecognizerModule::waitNextRecognition(int timeout)
{
    m_is_in_syncrecog = true;
    std::cout<<"Recognition: blocking mode on";
    list< pair<string, double> > recognitionResults;

    bool gotSomething = false;
    double endTime = Time::now() + timeout/1000.0;
    while(Time::now()<endTime && !gotSomething)
    {
        //std::cout<<".";
        const float ConfidenceThreshold = 0.3f;
        SPEVENT curEvent;
        ULONG fetched = 0;
        HRESULT hr = S_OK;

        m_cpRecoCtxt->GetEvents(1, &curEvent, &fetched);

        while (fetched > 0)
        {
            gotSomething = true;
            ISpRecoResult* result = reinterpret_cast<ISpRecoResult*>(curEvent.lParam);

            
            //Convert the catched sentence to strings. 
            CSpDynamicString dstrText;
            result->GetText(SP_GETWHOLEPHRASE, SP_GETWHOLEPHRASE, TRUE, &dstrText, NULL);
            string fullSentence = ws2s(dstrText);
            vector<string> words = split(fullSentence,' ');
            for(int w=0;w<words.size();w++)
            {
                //Todo extract the confidence value somehow...
                recognitionResults.push_back(make_pair(words[w], -1.0));
            }
                   
            //SPPHRASE* pPhrase = NULL; 
            ////Do some smarter stuff with the recognition
            //if (SUCCEEDED(result->GetPhrase(&pPhrase)))
            //{   
            //    if ((pPhrase->pProperties != NULL) && (pPhrase->pProperties->pFirstChild != NULL))
            //    {
            //        const SPPHRASEPROPERTY* pSemanticTag = pPhrase->pProperties->pFirstChild;
            //        if (pSemanticTag->SREngineConfidence > ConfidenceThreshold)
            //        {
            //            //std::cout<<pSemanticTag->pszValue<<std::endl;
            //        }
            //    }
            //    ::CoTaskMemFree(pPhrase);
            //}
            m_cpRecoCtxt->GetEvents(1, &curEvent, &fetched);
        }
    }
    std::cout<<"Recognition: blocking mode off : ";
    m_is_in_syncrecog = false;
    if (recognitionResults.size() == 0)
        recognitionResults.push_back( pair<string,double>("-1",-1.0));
    for(list<pair<string,double> >::iterator it= recognitionResults.begin();it != recognitionResults.end(); it++)
    {
        cout<<it->first<<" ("<<it->second<<") ";
    }
    cout<<endl;
    return recognitionResults;
}

/************************************************************************/
void SpeechRecognizerModule::say(const string &s)
{
    cout<<"TTS: "<<s<<endl;
    Bottle b;
    b.addString(s.c_str());
    m_port2iSpeak.write(b);
}

/************************************************************************/
bool  SpeechRecognizerModule::setGrammarCustom(CComPtr<ISpRecoGrammar> grammarToModify, string grammar, bool append)
{
    //Clear the existing runtime grammar
    SPSTATEHANDLE runtimeRootRule;
    bool everythingIsFine = true;
    everythingIsFine &= SUCCEEDED(grammarToModify->SetGrammarState(SPGS_DISABLED));
    everythingIsFine &= SUCCEEDED(grammarToModify->GetRule(L"rootRule", NULL, SPRAF_TopLevel | SPRAF_Active, TRUE, &runtimeRootRule));
    if(!append)
        everythingIsFine &= SUCCEEDED(grammarToModify->ClearRule(runtimeRootRule));   
    
    //Build a rule for each vocabulory
    map<string, SPSTATEHANDLE> vocabRules;
    for(map<string, list<string> >::iterator vIt = m_vocabulories.begin(); vIt != m_vocabulories.end(); vIt++)
    {
        //Get the rule name from the key in the dictionary (i.e Agent, Action, etc...)
        std::wstring tmp = s2ws(vIt->first);
        LPCWSTR cwRuleName = tmp.c_str();

        //Get the rule or create it
        everythingIsFine &= SUCCEEDED(grammarToModify->GetRule(cwRuleName, NULL, SPRAF_Dynamic, TRUE, &vocabRules[vIt->first]));
        everythingIsFine &= SUCCEEDED(grammarToModify->ClearRule(vocabRules[vIt->first]));
        for(list<string>::iterator wordIt = vIt->second.begin() ; wordIt != vIt->second.end(); wordIt++)
        {
            std::wstring wordTmp = s2ws(*wordIt);
            LPCWSTR cwWord = wordTmp.c_str();
            everythingIsFine &= SUCCEEDED( grammarToModify->AddWordTransition(vocabRules[vIt->first], NULL, cwWord, NULL, SPWT_LEXICAL, 1, NULL) );
        }
    }

    //Go through the given string and build the according grammar
    //Split the choices
    vector<string> sentences = split(grammar,'|');
    for(vector<string>::iterator it = sentences.begin() ; it != sentences.end() ; it++)
    {
        //Split the words
        vector<string> words = split(*it,' ');       
        SPSTATEHANDLE beforeWordHandle = runtimeRootRule;
        SPSTATEHANDLE afterWordHandle;
        for(vector<string>::iterator itWord = words.begin() ; itWord != words.end() ; itWord++)
        {                     
            if ((*itWord)!="")
            {         
                everythingIsFine &= SUCCEEDED(grammarToModify->CreateNewState(beforeWordHandle, &afterWordHandle));
                //Check if the current word is the name of a vocabulory
                if ( (*itWord)[0] == '#' && m_vocabulories.find(*itWord) != m_vocabulories.end())
                {
                    everythingIsFine &= SUCCEEDED(grammarToModify->AddRuleTransition(beforeWordHandle, afterWordHandle, vocabRules[*itWord], 1, NULL));
                }
                else
                {            
                    std::wstring wordTmp = s2ws(*itWord);
                    LPCWSTR cwWord = wordTmp.c_str();
                    everythingIsFine &= SUCCEEDED( grammarToModify->AddWordTransition(beforeWordHandle, afterWordHandle, cwWord, NULL, SPWT_LEXICAL, 1, NULL) );
                }
                beforeWordHandle = afterWordHandle;
            }
        }
        everythingIsFine &= SUCCEEDED( grammarToModify->AddWordTransition(beforeWordHandle, NULL, NULL, NULL, SPWT_LEXICAL, 1, NULL) );
    }
    everythingIsFine &= SUCCEEDED(grammarToModify->Commit(NULL));
    everythingIsFine &= SUCCEEDED(grammarToModify->SetGrammarState(SPGS_ENABLED));        
    everythingIsFine &= SUCCEEDED(grammarToModify->SetRuleState(NULL, NULL, SPRS_ACTIVE));
    everythingIsFine &= SUCCEEDED(m_cpRecoCtxt->Resume(0));

    if (!everythingIsFine)
    {
        cout<<">>>Problem while setting the custom grammar."<<endl;
    }
    return everythingIsFine;
}