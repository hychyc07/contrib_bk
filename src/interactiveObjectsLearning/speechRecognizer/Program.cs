#region Documentation
/** 
* 
\defgroup icub_speechRecognizer Process to speech recognition, 
          synthesis and online vocabulory / grammar management.
@ingroup icub_interactiveObjectsLearning 

\section intro_sec Description
The module is in c#, based on Microsoft Speech API. It is made to recognize speech based by receiving at runtime a grammar in different formats:
 * XML W3C specification
 * "custom" simple format closed to the one used in RAD.
 The class RobotGrammarManager maintains a dictionary of various vocabulories usefull in HRI (Objects, Agents, Actions...) which can be
 expanded at runtime and can be used in grammars.

\section lib_sec Libraries
Yarp C# Wrapper (/yarp2/example/swig), System.Speech (available by default in .NET framework 4)

\section parameters_sec Parameters
No parameters
 
\section portsa_sec Ports Accessed
If iSpeak is running then all text to speech commands will be forwarded to iSpeak.

\section portsc_sec Interface: Ports Created and data format
/modulename/recog/continuous:o broadcast commands recognized by the asynchronous grammar (see rpc commands "asynrecog" )
/moduleName/rpc handles rpc commands of type
 * "tts" "sentence to say"
    Say a sentence
 
 * "lang" "newLanguage"
    Set the language (e.g: en-us or fr-fr) of the speech recognizer and (if possible of the TTS). Only languages installed on the executing windows machine are available.

 * "asyncrecog"
    ** "get"
    Return the current asynchronous grammar
 
    ** "addGrammar"...
    Add a simple grammar (uses current state of RGM) to the asynchronous grammar
 
    ** "clear"...
    Reset the asynchrnous grammar to its original state
 
 * "recog"
    
    ** "timeout" "ms"
    Set the timeout for all recognition actions
 
    ** "choices" "choice1" "choice2"...
    Run a recognition over the list of choices passed in arguments. Returns the word/sentence recognized
    
    ** "dictation"
    Run a recognition of open dictation
 
    ** "grammarXML" "xml formatted grammar"
    Run the recognition of a grammar specified in the W3C SRGS xml format (http://www.w3.org/TR/speech-grammar/)
 
    ** "grammarSimple" "grammar to recognize"
    Run a recognition on a custom format using RobotGrammarManager, refer to RobotGrammarSpecification.txt

 * "RGM" "vocabulory"
    ** "get" "vocabuloryName"
    return a string containing all the vocabulory words
   
    ** "add" "vocabuloryName" "word"
    Expand a given vocabulory with a specific word
 
    ** "addAuto" "vocabuloryName"
    Expand a given vocabulory by putting the system in dictation mode and asking confirmation of what is being recognized

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None.
 
\section conf_file_sec Configuration Files
None.

\section tested_os_sec Tested OS
Windows 7. 

\author Stephane Lallee 

This file can be edited at icub/contrib/src/interactiveObjectsLearning/speechRecognizer/program.cs.
**/
#endregion

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Speech.Recognition;

namespace RobotSpeech
{
    class Program
    {

        static SpeechRecognizerServer m_speechRecognizer;


        static void Main(string[] args)
        {
            m_speechRecognizer = new SpeechRecognizerServer("speechRecognizer");
            m_speechRecognizer.Say("Speech Recognizer is running.");
            m_speechRecognizer.OpenGui();
        }


    }
}
