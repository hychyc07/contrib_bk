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

#ifndef MODULE_GENERATOR_H
#define MODULE_GENERATOR_H

#include <QDialog>
#include <QDir>

namespace Ui
{
    class ModuleGenerator;
}

/*! Module Generator */
class ModuleGenerator : public QDialog
{
    Q_OBJECT

public:
    ModuleGenerator();
    ~ModuleGenerator();

private:
    Ui::ModuleGenerator *ui;
    QDir aquilaRootDir;
    QDir aquilaSrcDir;
    QDir aquilaIncludeDir;
    QDir aquilaUiDir;
    QDir modulesRootDir;
    QDir moduleDir;
    QDir moduleSrcDir;
    QDir moduleConfDir;
    QDir moduleIncludeDir;
    QDir moduleGeneratorDir;
    QDir moduleGeneratorWorkingDir;
    QString moduleName;
    QString developerName;
    QStringList file;
    QStringList tmpFile;
    QStringList oldKeyword;
    QStringList newKeyword;
    QStringList backupFile;

    int existingFilesToUpdate;

    void cleanUp();
    bool initialise();
    bool createWorkingDirectories();
    bool createModuleDirectories();
    bool generateFiles();
    bool updateProject();
    bool removeDirectory(const QString &dirName);
    bool getFileNames(QString directory);

private slots:
    void generateNewModule();
};

#endif//MODULE_GENERATOR_H
