using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Speech.Recognition;

namespace RobotSpeech
{
    public class RobotGrammarManager
    {
        /// <summary>
        /// Contains all vocabulories (#Object, #Agent, #Action by default)
        /// </summary>
        Dictionary<string, List<string>> m_vocabulories = new Dictionary<string, List<string>>();
        public event EventHandler eventVocabuloryUpdated;

        Dictionary<string, GrammarBuilder> m_grammars = new Dictionary<string, GrammarBuilder>();
        System.Globalization.CultureInfo m_culture = System.Globalization.CultureInfo.CurrentCulture;

        public RobotGrammarManager()
        {
        }

        public void InitialiseVocabulories()
        {
            m_vocabulories.Add("#Object", new List<string>());
            m_vocabulories.Add("#Agent", new List<string>());
            m_vocabulories.Add("#Action", new List<string>());
            Console.WriteLine("Robot Grammar Manager : Vocabulories initialised.");
            foreach (KeyValuePair<string, List<string>> c in m_vocabulories)
            {
                Console.WriteLine(c.Key + '\t' + "==" + '\t' + new Choices(c.Value.ToArray()).ToGrammarBuilder().DebugShowPhrases);
            }
        }

        public Choices GetVocabulory(string vocabName)
        {
            if (m_vocabulories.ContainsKey(vocabName))
                return new Choices( m_vocabulories[vocabName].ToArray() );
            else
                return new Choices();
        }

        public bool AppendToVocabulory(string vocabName, string sentence)
        {
            if (!vocabName.StartsWith("#") || vocabName == "#Dictation" || vocabName == "#WildCard")
            {
                Console.WriteLine("Vocabulories have to start with a #. #Dictation and #WildCard are reserved. Aborting.");
                return false;
            }

            if (!m_vocabulories.ContainsKey(vocabName))
                m_vocabulories.Add(vocabName, new List<string>(){sentence});
            else if (!m_vocabulories[vocabName].Contains(sentence))
            {
                m_vocabulories[vocabName].Add(sentence);
                Console.WriteLine("Vocabulory " + vocabName + " augmented : " + new Choices(m_vocabulories[vocabName].ToArray()).ToGrammarBuilder().DebugShowPhrases);
            }
            if (eventVocabuloryUpdated != null)
                eventVocabuloryUpdated(this, null);

            return true;
        }

        public bool AppendToGrammars(string grammarName, string grammarDef)
        {
            throw new NotImplementedException();
        }

        public GrammarBuilder GetGrammar_Custom(string grammar)
        {
            Choices globalChoices = new Choices();
            string[] sentences = grammar.Split('|');
            foreach (string s in sentences)
            {
                GrammarBuilder sentenceBuilder = new GrammarBuilder();
                string[] words = s.Split(' ');
                foreach (string w in words)
                {
                    if (m_vocabulories.ContainsKey(w))
                        sentenceBuilder.Append(new Choices(m_vocabulories[w].ToArray()));
                    else if (w == "#Dictation")
                        sentenceBuilder.AppendDictation();
                    else if (w == "#WildCard")
                        sentenceBuilder.AppendWildcard();
                    else if (w != "")
                        sentenceBuilder.Append(w);
                }
                globalChoices.Add(sentenceBuilder);
            }

            GrammarBuilder globalBuilder = new GrammarBuilder(globalChoices);
            globalBuilder.Culture = m_culture;
            Console.WriteLine(globalBuilder.DebugShowPhrases);
            return globalBuilder;
        }

        public void SetLanguage(string cultureName)
        {
            System.Globalization.CultureInfo culture;
            try
            {
                culture = System.Globalization.CultureInfo.GetCultureInfoByIetfLanguageTag(cultureName);
            }
            catch
            {
                Console.WriteLine("Culture info is not found.");
                return;
            }

            m_culture = culture;
        }

        public string Help()
        {
            string help = "Robot Grammar Manager Help \n";
            help += "Build dynamic grammars of the form #Object, #Agent, #Action \n";
            help += "You can add a wildCard node with the #WildCard reference";
            help += "You can add a dictation node with the #Dictation reference";
            return help;
        }
    }
}
