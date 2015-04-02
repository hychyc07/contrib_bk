using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Speech.Recognition;
using System.Speech.Synthesis;
using System.IO;

namespace RobotSpeech
{
   public class SpeechRecognizerServer
   {
       #region TTS
       System.Speech.Synthesis.SpeechSynthesizer m_tts;
       Port m_portISpeak;
       #endregion

       #region One Shot Recognitions
       System.Speech.Recognition.SpeechRecognitionEngine m_reco;
       System.Speech.Recognition.Grammar m_grammar_dictation;
       System.Speech.Recognition.Grammar m_grammar_dictation_spelling;
        System.Speech.Recognition.Grammar m_grammar_simple;
       #endregion

        #region Asynchronous Recognition
        System.Speech.Recognition.SpeechRecognitionEngine m_reco_continuous;
        System.Speech.Recognition.GrammarBuilder m_grammar_continuous;
        Port m_portContinuousRecognition;
        #endregion

        #region RGM System
        public RobotGrammarManager m_grammarManager;
        #endregion

        #region RPC & Management
        System.Threading.Thread m_rpcThread;
        Port m_rpcPort;
        int m_defaultTimeout;
        string m_moduleName = "speechRecognizer";
        int m_trialsBeforeSpelling = 0;
        #endregion

        public static System.Globalization.CultureInfo myLanguage = System.Globalization.CultureInfo.GetCultureInfo("EN-us");
        SpeechRecognizerGUI m_speechRecognizerGui;

        public SpeechRecognizerServer(string moduleName)
        {

            System.Collections.ObjectModel.ReadOnlyCollection<RecognizerInfo> installedRecognizers = SpeechRecognitionEngine.InstalledRecognizers();
            
            //Synchronous Recognition
            m_reco = new System.Speech.Recognition.SpeechRecognitionEngine(myLanguage);

            Network.init();
            m_moduleName = moduleName;

            //TTS
            m_tts = new System.Speech.Synthesis.SpeechSynthesizer();
            m_portISpeak = new Port();
            m_portISpeak.open("/" + moduleName + "/tts/iSpeak:o");
            Network.connect("/" + moduleName + "/tts/iSpeak:o", "/iSpeak");

            //Grammars
            GrammarBuilder dictation = new GrammarBuilder();
            dictation.Culture = myLanguage;
            dictation.AppendDictation();
            m_grammar_dictation = new Grammar(dictation);
            GrammarBuilder spelling = new GrammarBuilder();
            spelling.Culture = myLanguage;
            spelling.AppendDictation("spelling");
            m_grammar_dictation_spelling = new Grammar(spelling);
            m_grammar_continuous = new GrammarBuilder("For sure this non empty grammar will never be recognized.");

            m_reco.SetInputToDefaultAudioDevice();
            m_reco.LoadGrammar(m_grammar_dictation);

            //Continuous Recognition
            m_reco_continuous = new SpeechRecognitionEngine();
            m_reco_continuous.SetInputToDefaultAudioDevice();
            m_portContinuousRecognition = new Port();
            m_portContinuousRecognition.open("/" + moduleName + "/recog/continuous:o");
            m_reco_continuous.LoadGrammar(new Grammar(m_grammar_continuous));
            m_reco_continuous.RecognizeCompleted += onContinuousRecognitionResult;
            m_reco_continuous.RecognizeAsync();

            m_grammarManager = new RobotGrammarManager();
            m_grammarManager.InitialiseVocabulories();
            SetLanguage("EN-us");
            //SetLanguage("fr-fr");

            Console.WriteLine("#########################");
            Console.WriteLine("#    Speech Recognizer  #");
            Console.WriteLine("#########################");

            Network.init();
            m_rpcPort = new Port();
            m_rpcPort.open("/" + m_moduleName + "/rpc");
            m_rpcThread = new System.Threading.Thread(HandleRPC);
            m_rpcThread.Start();
        }

        public void OpenGui()
        {
            m_speechRecognizerGui = new SpeechRecognizerGUI(this);
            m_speechRecognizerGui.ShowDialog();
            m_speechRecognizerGui = null;
            Close();
        }

        public System.Globalization.CultureInfo GetLanguage()
        {
            return myLanguage;
        }

        public bool SetLanguage(string cultureName)
        {
            //System.Globalization.CultureInfo[] cultures = System.Globalization.CultureInfo.GetCultures(System.Globalization.CultureTypes.AllCultures);
            System.Globalization.CultureInfo culture;
            try
            {
                culture = System.Globalization.CultureInfo.GetCultureInfoByIetfLanguageTag(cultureName);
            }
            catch
            {
                Console.WriteLine("Culture info is not found.");
                return false;
            }
            myLanguage = culture;

            System.Collections.ObjectModel.ReadOnlyCollection<InstalledVoice> voices = m_tts.GetInstalledVoices(culture);
            if (voices.Count > 0)
                m_tts.SelectVoice(voices.First().VoiceInfo.Name);

            m_reco = new System.Speech.Recognition.SpeechRecognitionEngine(culture);

            m_reco.SetInputToDefaultAudioDevice();
            GrammarBuilder dictation = new GrammarBuilder();
            dictation.Culture = myLanguage;
            dictation.AppendDictation();
            m_grammar_dictation = new Grammar(dictation);

            m_reco.LoadGrammar(m_grammar_dictation);

            m_reco_continuous.RecognizeCompleted -= onContinuousRecognitionResult;
            m_reco_continuous.RecognizeAsyncCancel();
            //m_reco_continuous.RecognizeAsyncStop();
            m_reco_continuous = new SpeechRecognitionEngine(culture);
            m_reco_continuous.SetInputToDefaultAudioDevice();
            m_grammar_continuous.Culture = culture;
            m_reco_continuous.LoadGrammar(new Grammar(m_grammar_continuous));
            m_reco_continuous.RecognizeCompleted += onContinuousRecognitionResult;
            m_reco_continuous.RecognizeAsync();

            m_grammarManager.SetLanguage(cultureName);

            Console.WriteLine("The culture has been set to " + cultureName);
            return true;
        }

        public void Say(string what, bool blocking = true)
        {
            Console.WriteLine("TTS : " + what);
            Bottle bTTS = new Bottle();
            bTTS.clear();
            bTTS.addString(what);
            m_portISpeak.write(bTTS);
            if (blocking)
                m_tts.Speak(what);
            else
                m_tts.SpeakAsync(what);
        }

        public string LearnNewWord(string vocabulory)
        {
            Say("I will learn a new " + vocabulory.Replace('#', ' ') + ".");
            string word = null;
            bool wordIsOk = false;

            int trials = 0;

            while (!wordIsOk)
            {
                word = null;
                RecognitionResult results = null;
                if (trials >= m_trialsBeforeSpelling)
                {
                    Say("Could you spell the new word for me?");
                    results = Recog_Spelling(m_defaultTimeout);


                    if (results != null)
                    {
                        word = "";
                        foreach (RecognizedWordUnit w in results.Words)
                            word += w.Text;
                    }
                }
                else
                {
                    Say("Please tell me the word.");
                    results = Recog_Dictation(m_defaultTimeout);
                    if (results != null)
                        word = results.Text;
                }
                trials++;
                if (word != null)
                {
                    RecognitionResult confirm = null;
                    while (confirm == null)
                    {
                        Say("Did you say " + word + "?");
                        confirm = Recog_Choices(new List<string>() { "Yes I did", "No I did not" }, m_defaultTimeout);
                    }

                    if (confirm.Words.First().Text == "Yes")
                        wordIsOk = true;
                }
                else
                {
                    Say("I didn't understand anything.");
                }
            }
            Say("Thanks, now I can use the word " + word + ".");
            m_grammarManager.AppendToVocabulory(vocabulory, word);
            return word;
        }

        public RecognitionResult Recog_Dictation(int timeout)
        {
            Console.WriteLine("Recognition Dictation loaded.");
            m_reco.UnloadAllGrammars();
            m_reco.LoadGrammar(m_grammar_dictation);
            RecognitionResult result = m_reco.Recognize(TimeSpan.FromMilliseconds(timeout));
            if (result != null)
                Console.WriteLine("Recognized : " + result.Text);
            return result;
        }

        public RecognitionResult Recog_Spelling(int timeout)
        {
            Console.WriteLine("Recognition Speeling loaded.");
            m_reco.UnloadAllGrammars();
            m_reco.LoadGrammar(m_grammar_dictation_spelling);
            RecognitionResult result = m_reco.Recognize(TimeSpan.FromMilliseconds(timeout));
            if (result != null)
                Console.WriteLine("Recognized : " + result.Text);
            return result;
        }

        public RecognitionResult Recog_Choices(List<string> choices, int timeout)
        {
            Console.Write("Recognition Choices Grammar loaded : ");

            foreach (string s in choices)
                Console.Write(s);
            Console.WriteLine();
            Choices c = new Choices(choices.ToArray());
            GrammarBuilder cBuild = c.ToGrammarBuilder();
            cBuild.Culture = m_reco.RecognizerInfo.Culture;
            m_reco.UnloadAllGrammars();
            m_grammar_simple = new Grammar(cBuild);

            m_reco.LoadGrammar(m_grammar_simple);
            RecognitionResult result = m_reco.Recognize(TimeSpan.FromMilliseconds(timeout));
            if (result != null)
                Console.WriteLine("Recognized : " + result.Text);
            else
                Console.WriteLine("Recognize Failure.");
            return result;
        }

        public RecognitionResult Recog_Grammar_XML(string grammarDef, int timeout)
        {
            Console.WriteLine("Recognition XML loaded.");
            string xmlFormattedGrammar = "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?> <grammar xmlns=\"http://www.w3.org/2001/06/grammar\" xml:lang=\""+m_reco.RecognizerInfo.Culture.Name+"\" version=\"1.0\">";
            //grammarDef += "<rule id=\"request\" scope=\"public\">Je veux acheter de la <one-of> <item>vodka</item><item>tequila</item></one-of></rule>";
            xmlFormattedGrammar += grammarDef;
            xmlFormattedGrammar += "</grammar>";

            byte[] byteArray = Encoding.ASCII.GetBytes(xmlFormattedGrammar);
            MemoryStream stream = new MemoryStream( byteArray );
            m_grammar_simple = new Grammar(stream, "request");
            m_reco.UnloadAllGrammars();
            m_reco.LoadGrammar(m_grammar_simple);

            RecognitionResult result = m_reco.Recognize(TimeSpan.FromMilliseconds(timeout));
            if (result != null)
                Console.WriteLine("Recognized : " + result.Text);
            else
                Console.WriteLine("Recognize Failure.");
            return result;
        }

        public RecognitionResult Recog_Grammar(Grammar g, int timeout)
        {
            m_reco.UnloadAllGrammars();
            Console.WriteLine("Recognition grammar loaded.");
            m_reco.LoadGrammar(g);

            RecognitionResult result = m_reco.Recognize(TimeSpan.FromMilliseconds(timeout));
            if (result != null)
                Console.WriteLine("Recognized : " + result.Text);
            else
                Console.WriteLine("Recognize Failure.");
            return result;
        }

        public RecognitionResult Recog_Grammar(List<Grammar> g, int timeout)
        {
            m_reco.UnloadAllGrammars();
            foreach (Grammar gg in g)
            {
                Console.WriteLine("Recognition grammar loaded.");
                m_reco.LoadGrammar(gg);
            }
            RecognitionResult result = m_reco.Recognize(TimeSpan.FromMilliseconds(timeout));
            if (result != null)
                Console.WriteLine("Recognized : " + result.Text);
            else
                Console.WriteLine("Recognize Failure.");
            return result;
        }

        public RecognitionResult Recog_Grammar(GrammarBuilder g, int timeout)
        {
            return Recog_Grammar(new List<Grammar>() { new Grammar(g) },timeout);
        }

        public RecognitionResult Recog_Grammar(List<GrammarBuilder> g, int timeout)
        {
            List<Grammar> gG = new List<Grammar>();
            foreach (GrammarBuilder gg in g)
            {
                gG.Add(new Grammar(gg));
            }
            return Recog_Grammar(gG, timeout);
        }
        public RecognitionResult Recog_Grammar(string RADStyle, int timeout)
        {
            return Recog_Grammar(m_grammarManager.GetGrammar_Custom(RADStyle), timeout);
        }
        private void onContinuousRecognitionResult(object sender, RecognizeCompletedEventArgs args)
        {
            //m_reco_continuous.RecognizeAsyncCancel();

            if (args.Result != null)
            {
                Bottle bReco = new Bottle();
                bReco.clear();
                //Console.WriteLine("Continuous reco : " + args.Result.Text);
                foreach (RecognizedWordUnit word in args.Result.Words)
                {
                    bReco.addString(word.Text);
                    bReco.addDouble(word.Confidence);
                }
                m_portContinuousRecognition.write(bReco);
                Console.WriteLine("Async. Recog : " + bReco.toString_c().c_str());
            }
            m_reco_continuous.RecognizeAsync();
        }

        delegate void GUICloser();
        private void CloseGUI()
        {
            if (m_speechRecognizerGui.InvokeRequired)
            {
                GUICloser d = new GUICloser(CloseGUI);
                m_speechRecognizerGui.Invoke(d);
            }
            else
            {
                m_speechRecognizerGui.Close();
                m_speechRecognizerGui = null;
            }
        }

        public void Close()
        {
            if (m_speechRecognizerGui != null)
                CloseGUI();
            m_reco_continuous.RecognizeCompleted -= onContinuousRecognitionResult;
            m_rpcThread.Abort();
            m_rpcPort.interrupt();
            m_rpcPort.close();
            m_portISpeak.interrupt();
            m_portISpeak.close();
            m_portContinuousRecognition.interrupt();
            m_portContinuousRecognition.close();

        }

        void HandleRPC()
        {
            while (m_rpcThread.IsAlive)
            {
                Bottle cmd = new Bottle();
                Bottle reply = new Bottle();
                reply.clear();
                reply.addString("ACK");
                m_rpcPort.read(cmd, true);
                string firstVocab = cmd.get(0).asString().c_str();
                switch (firstVocab)
                {
                    case "quit":
                        Close();
                        break;

                    #region RGM
                    //RobotGrammarManager
                    case "RGM":
                        {
                            string secondVocab = cmd.get(1).asString().c_str();
                            switch (secondVocab)
                            {
                                case "vocabulory":
                                    {
                                        string thirdVocab = cmd.get(2).asString().c_str();
                                        switch (thirdVocab)
                                        {
                                            case "get":
                                                {
                                                    string vocabName = cmd.get(3).asString().c_str();
                                                    string words = m_grammarManager.GetVocabulory(vocabName).ToGrammarBuilder().DebugShowPhrases;

                                                    //replace "smart quotes"
                                                    words = words.Replace('\u2018', '\'').Replace('\u2019', '\'').Replace('\u201c', '\"').Replace('\u201d', '\"');
                                                    words = words.Replace("[", "").Replace("]", "").Replace(",", " ").Replace("\'", "");
                                                    string[] wordsL = words.Split(' ');
                                                    foreach (string w in wordsL)
                                                        reply.addString(w);
                                                    break;
                                                }

                                            case "add":
                                                {
                                                    string vocabulory = cmd.get(3).asString().c_str();
                                                    string word = cmd.get(4).asString().c_str();
                                                    if (m_grammarManager.AppendToVocabulory(vocabulory, word))
                                                        reply.addString("OK");
                                                    else
                                                        reply.addString("ERROR");
                                                    break;
                                                }

                                            case "addAuto":
                                                {
                                                    string vocabulory = cmd.get(3).asString().c_str();
                                                    string word = LearnNewWord(vocabulory);
                                                    if (m_grammarManager.AppendToVocabulory(vocabulory, word))
                                                        reply.addString(word);
                                                    else
                                                        reply.addString("ERROR");
                                                    break;
                                                }

                                            default:
                                                {
                                                    reply.addString("UNKNOWN");
                                                    break;
                                                }
                                        }
                                        break;
                                    }

                                default:
                                    {
                                        reply.addString("UNKNOWN");
                                        break;
                                    }
                            }
                            break;
                        }
                    #endregion

                    #region Language
                    case "lang":
                        {
                            string lang = cmd.get(1).asString().c_str();
                            bool result = SetLanguage(lang);
                            SetLanguage(lang);
                            if (result)
                                reply.addString("OK");
                            else
                                reply.addString("Language not installed.");
                            break;//TTS
                        }
                    #endregion

                    #region TTS
                    case "tts":
                        {
                            string sentence = cmd.get(1).asString().c_str();
                            Say(sentence);
                            reply.addString("OK");
                            break;//TTS
                        }
                    #endregion

                    #region Async Recog
                    case "asyncrecog":
                        {
                            string secondVocab = cmd.get(1).asString().c_str();
                            switch (secondVocab)
                            {
                                case "getGrammar":
                                    {
                                        reply.addString(m_grammar_continuous.DebugShowPhrases);
                                        break;
                                    }
                                case "addGrammar":
                                    {
                                        string RADStyle = cmd.get(2).asString().c_str();
                                        GrammarBuilder appendG = m_grammarManager.GetGrammar_Custom(RADStyle);
                                        //Choices bothChoices = new Choices(new GrammarBuilder[] { m_grammar_continuous, appendG });
                                        //m_grammar_continuous = bothChoices.ToGrammarBuilder();
                                        m_reco_continuous.LoadGrammar(new Grammar(appendG));
                                        Console.WriteLine("Added to continuous reco : " + appendG.DebugShowPhrases);
                                        reply.addString("Added");
                                        break;
                                    }
                                case "clear":
                                    {
                                        m_reco_continuous.UnloadAllGrammars();
                                        m_reco_continuous.LoadGrammar(new Grammar(m_grammar_continuous));
                                        Console.WriteLine("Cleared continuous reco.");
                                        reply.addString("Cleared");
                                        break;
                                    }
                            }
                            break;
                        }
                    #endregion

                    #region Recognition
                    case "recog":
                        {
                            string secondVocab = cmd.get(1).asString().c_str();
                            switch (secondVocab)
                            {
                                case "timeout":
                                    {
                                        int newTimeout = cmd.get(2).asInt();
                                        m_defaultTimeout = newTimeout;
                                        Console.WriteLine("Timeout for recognition changed to " + m_defaultTimeout);
                                        break;
                                    }

                                case "choices":
                                    {
                                        List<string> choices = new List<string>();
                                        for (int wI = 2; wI < cmd.size(); wI++)
                                        {
                                            choices.Add(cmd.get(wI).asString().c_str());
                                        }
                                        RecognitionResult result = Recog_Choices(choices, m_defaultTimeout);
                                        if (result != null)
                                        {
                                            foreach (RecognizedWordUnit word in result.Words)
                                            {
                                                reply.addString(word.Text);
                                                reply.addDouble(word.Confidence);
                                            }
                                        }
                                        else
                                            reply.addString("-1");
                                        break;
                                    }

                                case "dictation":
                                    {
                                        RecognitionResult result = Recog_Dictation(m_defaultTimeout);
                                        if (result != null)
                                        {
                                            foreach (RecognizedWordUnit word in result.Words)
                                            {
                                                reply.addString(word.Text);
                                                reply.addDouble(word.Confidence);
                                            }
                                        }
                                        else
                                            reply.addString("-1");
                                        break;
                                    }

                                case "grammarXML":
                                    {
                                        string xmlDescription = cmd.get(2).asString().c_str();
                                        RecognitionResult result = Recog_Grammar_XML(xmlDescription, m_defaultTimeout);
                                        if (result != null)
                                        {
                                            foreach (RecognizedWordUnit word in result.Words)
                                            {
                                                reply.addString(word.Text);
                                                reply.addDouble(word.Confidence);
                                            }
                                        }
                                        else
                                            reply.addString("-1");
                                        break;
                                    }

                                case "grammarSimple":
                                    {
                                        string RADStyle = cmd.get(2).asString().c_str();
                                        RecognitionResult result = Recog_Grammar(m_grammarManager.GetGrammar_Custom(RADStyle), m_defaultTimeout);
                                        if (result != null)
                                        {
                                            foreach (RecognizedWordUnit word in result.Words)
                                            {
                                                reply.addString(word.Text);
                                                reply.addDouble(word.Confidence);
                                            }
                                        }
                                        else
                                            reply.addString("-1");
                                        break;
                                    }

                                default:
                                    reply.addString("UNKNOWN");
                                    break;
                            }
                            break; //Recog
                        }
                    #endregion

                    default:
                        reply.addString("UNKNOWN");
                        break;
                }
                m_rpcPort.reply(reply);
            }
        }
    }
}
