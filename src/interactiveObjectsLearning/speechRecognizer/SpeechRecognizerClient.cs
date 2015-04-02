using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobotSpeech
{
    public class SpeechRecognizerClient
    {
        Port m_portSM;

        public SpeechRecognizerClient(string clientName, string serverName = "speechRecognizer")
        {
            Network.init();
            m_portSM = new Port();
            m_portSM.open("/" + clientName + "/speechRecognizer:rpc");
            while (!Network.connect("/" + clientName + "/speechRecognizer:rpc", "/" + serverName + "/rpc"))
            {
                Console.WriteLine("Connecting to Speech Manager Server...");
                Time.delay(0.1);
            }
            Console.WriteLine("Connected.");
        }

        public void Close()
        {
            m_portSM.interrupt();
            m_portSM.close();
        }

        public void Say(string what)
        {
            Bottle cmd = new Bottle();
            Bottle reply = new Bottle();
            reply.clear();
            cmd.clear();
            cmd.addString("tts");
            cmd.addString(what);
            m_portSM.write(cmd,reply);
        }

        public Bottle RecogSimpleGrammar(string what)
        {
            Bottle cmd = new Bottle();
            Bottle reply = new Bottle();
            reply.clear();
            cmd.clear();
            cmd.addString("recog");
            cmd.addString("grammarSimple");
            cmd.addString(what);
            m_portSM.write(cmd, reply);
            return reply;
        }

        public bool ExpandVocabulory(string vocabulory, string newWord)
        {
            Bottle cmd = new Bottle();
            Bottle reply = new Bottle();
            reply.clear();
            cmd.clear();
            cmd.addString("RGM");
            cmd.addString("vocabulory");
            cmd.addString("add");            
            cmd.addString(vocabulory);
            cmd.addString(newWord);
            m_portSM.write(cmd, reply);
            if (reply.get(1).asString().c_str() == "OK")
                return true;
            else
                return false;
        }

        public bool ExpandVocabulory(string vocabulory)
        {
            Bottle cmd = new Bottle();
            Bottle reply = new Bottle();
            reply.clear();
            cmd.clear();
            cmd.addString("RGM");
            cmd.addString("vocabulory");
            cmd.addString("addAuto");
            cmd.addString(vocabulory);
            m_portSM.write(cmd, reply);
            if (reply.get(1).asString().c_str() == "OK")
                return true;
            else
                return false;
        }

        public bool AsyncGrammarAdd(string g)
        {
            Bottle cmd = new Bottle();
            Bottle reply = new Bottle();
            reply.clear();
            cmd.clear();
            cmd.addString("asyncrecog");
            cmd.addString("addGrammar");
            cmd.addString(g);
            m_portSM.write(cmd, reply);
            if (reply.get(1).asString().c_str() != "ERROR")
                return true;
            else
                return false;
        }

        public void AsyncGrammarClear()
        {
            Bottle cmd = new Bottle();
            Bottle reply = new Bottle();
            reply.clear();
            cmd.clear();
            cmd.addString("asyncrecog");
            cmd.addString("clear");
            m_portSM.write(cmd, reply);
        }
    }
}
