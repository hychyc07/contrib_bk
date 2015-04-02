using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Speech;
using System.Speech.Recognition;
using System.Collections.ObjectModel;

namespace RobotSpeech
{
    public partial class SpeechRecognizerGUI : Form
    {
        SpeechRecognizerServer m_speechRecognizer;
        ReadOnlyCollection<RecognizerInfo> m_languagesInstalled;

        public SpeechRecognizerGUI(SpeechRecognizerServer speechRecognizer)
        {
            InitializeComponent();

            m_speechRecognizer = speechRecognizer;

            m_languagesInstalled = SpeechRecognitionEngine.InstalledRecognizers();
            foreach (RecognizerInfo lang in m_languagesInstalled)
            {
                listBoxLanguage.Items.Add(lang.Culture.EnglishName);
            }
        }

        private void buttonSay_Click(object sender, EventArgs e)
        {
            m_speechRecognizer.Say(textBoxSay.Text);
        }

        private void listBoxLanguage_SelectedIndexChanged(object sender, EventArgs e)
        {
            RecognizerInfo lang = m_languagesInstalled[listBoxLanguage.SelectedIndex];

            m_speechRecognizer.SetLanguage(lang.Culture.Name);

            MessageBox.Show("Speech recognition set to " + lang.Culture.EnglishName);
        }

    }
}
