//##############################################################################################################################################################################################################//
//Aquila - An Open-Source GPU-Accelerated Toolkit for Cognitive and Neuro-Robotics Research																														//
//																																																				//
//Copyright (c) <2012>, <Martin Peniak - www.martinpeniak.com>																																					//
//All rights reserved.																																															//
//																																																				//
//Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:																//
//																																																				//
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.																				//
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.	//
//																																																				//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR	//
//A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT	//
//LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR	//
//TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.																//
//                                                                                                                                                                                                              //
//The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted                                                                                  //
//as representing official policies,either expressed or implied, of the FreeBSD Project.                                                                                                                        //
//##############################################################################################################################################################################################################//
//Acknowledgement: This class is based on EcWin7 library by Emanuele Colombo

#if defined _WIN64 || defined _WIN32
#include "win7progressBar.h"
#include <Windows.h>
#endif

#ifdef NTDDI_WIN7

DEFINE_GUID(CLSID_TaskbarList,0x56fdf344,0xfd6d,0x11d0,0x95,0x8a,0x0,0x60,0x97,0xc9,0xa0,0x90);
DEFINE_GUID(IID_ITaskbarList3,0xea1afb91,0x9e28,0x4b86,0x90,0xE9,0x9e,0x9f,0x8a,0x5e,0xef,0xaf);

/*!
 * \brief Constructor.
 */
Win7ProgressBar::Win7ProgressBar()
{
    clients = 0;
}

/*!
 * \brief Initialisation.
 * \param[in] wid - window ID
 */
void Win7ProgressBar::init(WId wid)
{
    mWindowId = wid;
    mTaskbarMessageId = RegisterWindowMessage(LPCSTR("TaskbarButtonCreated"));
}

/*!
 * \brief Windows event handler callback.
 * \param[in] message - pointer to message
 * \param[out] result - pointer to result
 */
bool Win7ProgressBar::winEvent(MSG *message, long *result)
{
    if(message->message==mTaskbarMessageId)
    {
        HRESULT hr = CoCreateInstance(CLSID_TaskbarList, 0, CLSCTX_INPROC_SERVER, IID_ITaskbarList3, reinterpret_cast<void**> (&(mTaskbar)));
        *result = hr;
        return true;
    }
    return false;
}

/*!
 * \brief Sets progress bar current value.
 * \param[in] value - current value
 * \param[in] max - maximum value
 */
void Win7ProgressBar::setProgressValue(int value, int max)
{
    mTaskbar->SetProgressValue(mWindowId, value, max);
}

/*!
 * \brief Sets progress bar current state.
 * \param[in] state - current state (e.g. active, error, pause, ecc...)
 */
void Win7ProgressBar::setProgressState(ToolBarProgressState state)
{
    mTaskbar->SetProgressState(mWindowId, (TBPFLAG)state);
}

/*!
 * \brief Adds new client.
 * \return number of current clients
 */
int Win7ProgressBar::addClient()
{
    clients ++;

    clientProgress.push_back(0);
    qDebug(QString(QString(" - main_gui: added a new client to Win7 progress bar, returned ID: ")+ QString::number(clientProgress.size())+ QString(" ,connected clients: ")+ QString::number(clients)).toStdString().c_str());

    return clientProgress.size()-1;
}

/*!
 * \brief Removes client.
 * \param[in] id - identification number of the client
 */
void Win7ProgressBar::removeClient(int id)
{
    clients --;
    clientProgress.at(id) = 0;
    qDebug(QString(QString(" - main_gui: removing a client from Win7 progress bar, removed ID: ")+ QString::number(clientProgress.size())+ QString(" ,connected clients: ")+ QString::number(clients)).toStdString().c_str());
}

/*!
 * \brief Sets progress of specified client.
 * \param[in] id - identification number of the client
 * \param[in] progress - current client progress
 */
void Win7ProgressBar::setClientProgress(int id, int progress)
{
    clientProgress.at(id) = progress;
}

/*!
 * \brief Shows overall progress.
 */
void Win7ProgressBar::showOverallProgress()
{
    int overallProgress = 0;

    for(int i=0; i<(int)clientProgress.size(); i++)
    {
        overallProgress += clientProgress.at(i);
    }

    overallProgress = overallProgress / clients;
    setProgressValue(overallProgress, 100);

    qDebug(" - main_gui: clients: %i overall progess: %i", clients, overallProgress);
}

#endif
