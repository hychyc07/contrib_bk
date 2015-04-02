//##############################################################################################################################################################################################################//
//aquila - An Open-Source GPU-Accelerated Toolkit for Cognitive and Neuro-Robotics Research																														//
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

#include "moduleGenerator.h"
#include "ui_moduleGenerator.h"
#include <QMessageBox>
#include <QTextStream>

/*!
 * \brief Constructor.
 */
ModuleGenerator::ModuleGenerator() : QDialog(), ui(new Ui::ModuleGenerator)
{
    ui->setupUi(this);
    setWindowTitle("Module Generator");

    //connect slots/signals
    QObject::connect(ui->pushButton_generate, SIGNAL(clicked()), this, SLOT(generateNewModule()));
    QObject::connect(ui->pushButton_cancel, SIGNAL(clicked()), this, SLOT(reject()));

    //initialise directory names
    aquilaRootDir.setPath(getenv("AQUILA_ROOT"));
    aquilaIncludeDir.setPath(aquilaRootDir.path() + QDir::separator() + QString("aquila") + QDir::separator() + QString("include"));
    aquilaSrcDir.setPath(aquilaRootDir.path() + QDir::separator() + QString("aquila") + QDir::separator() + QString("src"));
    aquilaUiDir.setPath(aquilaRootDir.path() + QDir::separator() + QString("aquila") + QDir::separator() + QString("ui"));
    modulesRootDir.setPath(aquilaRootDir.path() + QDir::separator() + QString("modules"));
    moduleGeneratorDir.setPath(aquilaRootDir.path() + QDir::separator() + QString("aquila") + QDir::separator() + QString("res") + QDir::separator() + QString("moduleGenerator"));
    moduleGeneratorWorkingDir.setPath(moduleGeneratorDir.path() + QDir::separator() + QString("tmp"));

    //initialise keywords used in module template files
    oldKeyword<<"blank"<<"Blank"<<"BLANK"<<"YourName";

    //initialise line edit validators
    QRegExp moduleExp("[a-z]+[A-Z]+[a-z]+[A-Z]+[a-z]+");
    QRegExpValidator *moduleNameValidator = new QRegExpValidator(moduleExp, 0);
    ui->lineEdit_moduleName->setValidator(moduleNameValidator);
    QRegExp developerExp("[A-Z][a-z]+\\s[A-Z][a-z]+\\s[A-Z][a-z]+\\s[A-Z][a-z]+");
    QRegExpValidator *developerNameValidator = new QRegExpValidator(developerExp, 0);
    ui->lineEdit_developerName->setValidator(developerNameValidator);

    //initialise tooltips
    ui->lineEdit_moduleName->setToolTip("The module name need to start in lower case and continue in camel caps, e.g. yourModuleName.");
    ui->lineEdit_developerName->setToolTip("The developer forename and surname(s) need to start with capital letter(s).");
    ui->pushButton_generate->setToolTip("Generate new module with the module and developer name specified above.");

    //set focus to module line edit
    ui->lineEdit_moduleName->setFocus();
}

/*!
 * \brief Destructor.
 */
ModuleGenerator::~ModuleGenerator()
{
    delete ui;
}

/*!
 * \brief Generates new module.
 */
void ModuleGenerator::generateNewModule()
{
    if(initialise() && !QMessageBox::question(this, tr("New module"), tr("Do you really want to generate new module called ") + moduleName + QString("?"), tr("&Yes"), tr("&No"),QString::null, 0, 1 ))
    {
        //create temporary working directories
        createWorkingDirectories();

        //create directories for the new modules
        createModuleDirectories();

        //get the name of all the module template files and aquila files that need updating
        getFileNames(moduleGeneratorDir.path());

        //generate new files from templates
        generateFiles();

        //update project with the new files
        updateProject();

        //remove all temporary files and directories
        cleanUp();

        //show message letting user know that Aquila project needs to be rebuild
        QMessageBox::about(this, tr("Module generated"), tr("The new module was successfully generated and added to Aquila project. These changes will take effect after you re-run CMake and recompile the project."));

        //clear line edits,set focus and close dialog
        ui->lineEdit_moduleName->clear();
        ui->lineEdit_developerName->clear();
        ui->lineEdit_moduleName->setFocus();
        accept();
    }
}

/*!
 * \brief Checks if the requested module name is available.
 * \return true on success
 */
bool ModuleGenerator::initialise()
{
    qDebug("module_generator: checking if the requested name is available");

    //initialise directory names for the new module
    moduleName = ui->lineEdit_moduleName->text();
    moduleDir = modulesRootDir.path() + QDir::separator() + moduleName;
    moduleIncludeDir = moduleDir.path() + QDir::separator() + QString("include");
    moduleSrcDir = moduleDir.path() + QDir::separator() + QString("src");
    moduleConfDir = moduleDir.path() + QDir::separator() + QString("conf");

    //check if the name is available
    if(moduleName.isEmpty())
    {
        QMessageBox::warning(this, tr("Module name missing"), tr("The module name is missing, please enter the name and try again."));
        return false;
    }

    //check if the module name exists
    if(moduleDir.exists())
    {
        QMessageBox::warning(this, tr("Module name not available"), tr("The name you have requested is already taken, please change the name and try again."));
        return false;
    }
    else
    {
        qDebug("module_generator: the name is available");
    }

    //check if the developer name was entered
    developerName = ui->lineEdit_developerName->text();
    if(developerName.isEmpty())
    {
        if(QMessageBox::question(this, tr("Developer name missing"), tr("Developer name is missing, which is used in copyright credits section. Are you sure you want to continue without name?"), tr("&Yes"), tr("&No"),QString::null, 0, 1))
        {
            return false;
        }
        developerName = "YourName";
    }

    //initialise keywords
    QString tmp = moduleName;
    newKeyword.clear();
    newKeyword.push_back(moduleName);
    newKeyword.push_back(moduleName.at(0).toUpper() + tmp.remove(0,1));
    newKeyword.push_back(moduleName.toUpper());
    newKeyword.push_back(developerName);

    return true;
}

/*!
 * \brief Creates temporary working directories.
 * \note This function creates a temporary working directories that are used temporary saving of updated module template files.
 * \return true on success
 */
bool ModuleGenerator::createWorkingDirectories()
{
    //remove any old working directories
    if(moduleGeneratorWorkingDir.exists())
    {
        qDebug("module_generator: removing old working directories");
        removeDirectory(moduleGeneratorWorkingDir.path());
    }

    //create new working directories
    qDebug("module_generator: creating working directory at %s", moduleGeneratorWorkingDir.path().toStdString().c_str());
    bool success =  moduleGeneratorDir.mkpath(moduleGeneratorWorkingDir.path() + QDir::separator() + QString("GUI") + QDir::separator() + QString("include"))
                    && moduleGeneratorDir.mkpath(moduleGeneratorWorkingDir.path() + QDir::separator() + QString("GUI") + QDir::separator() + QString("src"))
                    && moduleGeneratorDir.mkpath(moduleGeneratorWorkingDir.path() + QDir::separator() + QString("GUI") + QDir::separator() + QString("ui"))
                    && moduleGeneratorDir.mkpath(moduleGeneratorWorkingDir.path() + QDir::separator() + QString("module") + QDir::separator() + QString("conf"))
                    && moduleGeneratorDir.mkpath(moduleGeneratorWorkingDir.path() + QDir::separator() + QString("module") + QDir::separator() + QString("examples"))
                    && moduleGeneratorDir.mkpath(moduleGeneratorWorkingDir.path() + QDir::separator() + QString("module") + QDir::separator() + QString("include"))
                    && moduleGeneratorDir.mkpath(moduleGeneratorWorkingDir.path() + QDir::separator() + QString("module") + QDir::separator() + QString("src"));
    if(success)
    {
        qDebug("module_generator: working directories were successfully created");
    }
    else
    {
        qCritical("module_generator: failed to create working directories");
        return false;
    }

    return true;
}

/*!
 * \brief Creates new module directories.
 * \return true on success
 */
bool ModuleGenerator::createModuleDirectories()
{
    qDebug("module_generator: creating module directories at %s", moduleDir.path().toStdString().c_str());
    bool success =  moduleDir.mkpath(moduleDir.path() + QDir::separator() + QString("src"))
                    && moduleDir.mkpath(moduleDir.path() + QDir::separator() + QString("include"))
                    && moduleDir.mkpath(moduleDir.path() + QDir::separator() + QString("conf"))
                    && moduleDir.mkpath(moduleDir.path() + QDir::separator() + QString("examples"));
    if(success)
    {
        qDebug("module_generator: module directories were successfully created");
    }
    else
    {
        qCritical("module_generator: failed to create module directories");
        return false;
    }

    return true;
}

/*!
 * \brief Generates all module and GUI files using existing templates.
 * \return true on success
 */
bool ModuleGenerator::generateFiles()
{
    //add these files to the existing list - these are the file that need adding new code to accomodate changes
    file.append(aquilaIncludeDir.path() + QDir::separator() + QString("mainWindow.h"));
    file.append(aquilaSrcDir.path() + QDir::separator() + QString("mainWindow.cpp"));
    file.append(modulesRootDir.path() + QDir::separator() + QString("CMakeLists.txt"));

    //initialise this variable with the number of files (above) that need adding new code
    existingFilesToUpdate = 3;

    //change the keywords in all the files
    for(int i=0; i<file.size(); i++)
    {
        //open source file
        QFile sourceFile(file.at(i));
        if(!sourceFile.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            qCritical("module_generator: failed to open source file: %s", file.at(i).toStdString().c_str());
            return false;
        }

        //open destination file
        QString destinationFilePath;
        if(i < file.size()-existingFilesToUpdate) //copies of template module files are opened in the previously created working folder subdirectories
        {
            QString rootFolderName = "moduleGenerator";
            destinationFilePath = file.at(i);
            destinationFilePath.remove(0, file.at(i).indexOf(rootFolderName) + rootFolderName.size());
            destinationFilePath = moduleGeneratorWorkingDir.path() + destinationFilePath.replace(oldKeyword.at(0),moduleName);
        }
        else //copies of other files that need adding code to accomodate changes are temporarily places in the root of working directory
        {
            QFileInfo fileInfo(file.at(i));
            destinationFilePath = moduleGeneratorWorkingDir.path() + QDir::separator() + fileInfo.fileName();
        }

        QFile destinationFile(destinationFilePath);
        if(!destinationFile.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            qCritical("module_generator: failed to open destination file: %s", destinationFile.fileName().toStdString().c_str());
            return false;
        }
        else
        {
            tmpFile.append(destinationFilePath);
        }

        //initialise file reader and writer
        QTextStream reader(&sourceFile);
        QTextStream writer(&destinationFile);
        QString line;

        //read from the source, make changes and save to destination file
        line = reader.readLine();
        while(!line.isNull())
        {
            if(i<file.size()-existingFilesToUpdate) //reaplce kewords with desired names
            {
                for(int j=0; j<oldKeyword.size(); j++)
                {
                    if(line.contains(oldKeyword.at(j)))
                    {
                        line.replace(oldKeyword.at(j), newKeyword.at(j));
                    }
                }
            }
            else //add new code
            {
                QString tmp;

                if(line.contains("//ModuleGenerator:write:0")) //add forward class declaration (mainWindow.h)
                {
                    tmp = line;
                    line.clear();
                    line = QString("class ") + newKeyword.at(1) + QString(";\n") + tmp;
                }
                else if(line.contains("ModuleGenerator:write:1")) //add new module array of pointers to instances (mainWindow.h)
                {
                    tmp = line;
                    line.clear();
                    line = QString("\t") + newKeyword.at(1) + QString(" *") + newKeyword.at(0) + QString("[MAX_MODULE_INSTANCES]") + QString(";\n") + tmp;
                }
                else if(line.contains("ModuleGenerator:write:2")) //adds new header file (mainWindow.cpp)
                {
                    tmp = line;
                    line.clear();
                    line = QString("#include \"") + newKeyword.at(0) + QString(".h\"\n") + tmp;
                }
                else if(line.contains("ModuleGenerator:write:3")) //add new module to the list of recognised modules
                {
                    tmp = line;
                    line = reader.readLine();
                    QString codeToUpdate = line;
                    codeToUpdate.replace(";",QString(QString("<<\"") + newKeyword.at(0) + QString("\";")));
                    codeToUpdate.prepend(tmp + QString("\n"));
                    line = codeToUpdate;
                }
                else if(line.contains("ModuleGenerator:write:4"))
                {
                    tmp = line;
                    line.clear();
                    line = QString("\t\telse if(module==\"") + newKeyword.at(0) + QString("\")\n\t\t{\n\t\t\t") + newKeyword.at(0) + QString("[instance] = new ") + newKeyword.at(1) + QString("(this, module, \"") + newKeyword.at(1) +  QString("\", server, instance, tabWidget->newTabID);\n\t\t\t") + QString("success = ") + newKeyword.at(0) + QString("[instance]->moduleConnected;\n\t\t}\n") + tmp;
                }
                else if(line.contains("ModuleGenerator:write:5"))
                {
                    tmp = line;
                    line.clear();
                    line = QString("\tadd_subdirectory(") + newKeyword.at(0) + QString(")\n") + tmp;
                }
                else if(line.contains("ModuleGenerator:write:6"))
                {
                    tmp = line;
                    line.clear();
                    line = QString("\tmessage(STATUS \" - ") + newKeyword.at(0) + QString("\")\n") + tmp;
                }
                else if(line.contains("ModuleGenerator:write:7"))
                {
                    tmp = line;
                    line.clear();
                    line = QString("\tmessage(\" - ") + newKeyword.at(0) + QString("\")\n") + tmp;
                }
            }

            //write changes to file
            writer<<line<<"\n";

            //read new line
            line = reader.readLine();
        }

        //close the files
        sourceFile.close();
        destinationFile.close();
    }

	return true;
}

/*!
 * \brief Updates project with the new files.
 * \return true on success
 */
bool ModuleGenerator::updateProject()
{
    QString destinationFile;

    for(int i=0; i<tmpFile.size(); i++)
    {
        //get file information
        QFileInfo fileInfo(tmpFile.at(i));

        if(i < tmpFile.size()-existingFilesToUpdate)
        {
            //get the destination file path
            if(tmpFile.at(i).contains(QString("GUI") + QDir::separator() + QString("include")))
            {
                destinationFile = aquilaIncludeDir.path() + QDir::separator() + fileInfo.fileName();
            }
            else if(tmpFile.at(i).contains(QString("GUI") + QDir::separator() + QString("src")))
            {
                destinationFile = aquilaSrcDir.path() + QDir::separator() + fileInfo.fileName();
            }
            else if(tmpFile.at(i).contains(QString("GUI") + QDir::separator() + QString("ui")))
            {
                destinationFile = aquilaUiDir.path() + QDir::separator() + fileInfo.fileName();
            }
            if(tmpFile.at(i).contains(QString("module") + QDir::separator() + QString("conf")))
            {
                destinationFile = moduleConfDir.path() + QDir::separator() + fileInfo.fileName();
            }
            if(tmpFile.at(i).contains(QString("module") + QDir::separator() + QString("include")))
            {
                destinationFile = moduleIncludeDir.path() + QDir::separator() + fileInfo.fileName();
            }
            else if(tmpFile.at(i).contains(QString("module") + QDir::separator() + QString("src")))
            {
                destinationFile = moduleSrcDir.path() + QDir::separator() + fileInfo.fileName();
            }
            else if(tmpFile.at(i).contains(QString("module") + QDir::separator() + QString("CMakeLists.txt")))
            {
                destinationFile = moduleDir.path() + QDir::separator() + fileInfo.fileName();
            }

            //copy file to destination file path
            if(QFile::copy(tmpFile.at(i), destinationFile))
            {
                qDebug("module_generator: copied %s to %s", tmpFile.at(i).toStdString().c_str(), destinationFile.toStdString().c_str());
            }
            else
            {
                qCritical("module_generator: failed to copy %s to %s", tmpFile.at(i).toStdString().c_str(), destinationFile.toStdString().c_str());
            }
        }
        else
        {
            if(tmpFile.at(i).contains("mainWindow.h"))
            {
                destinationFile = aquilaIncludeDir.path() + QDir::separator() + fileInfo.fileName();
            }
            else if(tmpFile.at(i).contains("mainWindow.cpp"))
            {
                destinationFile = aquilaSrcDir.path() + QDir::separator() + fileInfo.fileName();
            }
            else if(tmpFile.at(i).contains("CMakeLists.txt"))
            {
                destinationFile = modulesRootDir.path() + QDir::separator() + fileInfo.fileName();
            }
            else
            {
                qCritical("module_generator: %s was not recognised", tmpFile.at(i).toStdString().c_str());
                return false;
            }

            //back-up the original file name
            if(QFile::rename(destinationFile, destinationFile+QString("_backup")))
            {
                qDebug("module_generator: backed-up %s to %s", destinationFile.toStdString().c_str(), QString(destinationFile+QString("_backup")).toStdString().c_str());
                backupFile.append(destinationFile+QString("_backup"));

                //copy the new file name in place of the original file
                if(QFile::copy(tmpFile.at(i), destinationFile))
                {
                    qDebug("module_generator: copied %s to %s", tmpFile.at(i).toStdString().c_str(), destinationFile.toStdString().c_str());
                }
                else
                {
                    qCritical("module_generator: failed to copy %s to %s, restoring from backup", tmpFile.at(i).toStdString().c_str(), destinationFile.toStdString().c_str());
                    if(QFile::copy(destinationFile+QString("_backup"), destinationFile))
                    {
                        qDebug("module_generator: restoring successful");
                    }
                    else
                    {
                        qCritical("module_generator: restoring failed");
                        return false;
                    }
                }
            }
            else
            {
                qCritical("module_generator: failed to backup %s to %s", destinationFile.toStdString().c_str(), QString(destinationFile+QString("_backup")).toStdString().c_str());
                return false;
            }
        }
    }

    return true;
}

/*!
 * \brief Cleans up all temporary files and directories.
*/
void ModuleGenerator::cleanUp()
{
    qDebug("module_generator: cleaning up temporary directories and files");

    //delete temporary working directories
    if(removeDirectory(moduleGeneratorWorkingDir.path()))
    {
        qDebug("module_generator: working directory was successfully removed from %s", moduleGeneratorWorkingDir.path().toStdString().c_str());
    }
    else
    {
        qCritical("module_generator: failed to remove working directory from %s", moduleGeneratorWorkingDir.path().toStdString().c_str());
    }

    //remove all the backup files
    for(int i=0; i<backupFile.size(); i++)
    {
        if(aquilaRootDir.remove(backupFile.at(i)))
        {
            qDebug("module_generator: %s successfully removed", backupFile.at(i).toStdString().c_str());
        }
        else
        {
            qDebug("module_generator: failed to remove %s", backupFile.at(i).toStdString().c_str());
        }
    }
}

/*!
 * \brief Delete a directory along with all of its contents.
 * \param dirName path of directory to remove.
 * \return true on success
*/
bool ModuleGenerator::removeDirectory(const QString &dirName)
{
    bool result = true;
    QDir dir(dirName);

    if(dir.exists(dirName))
    {
        Q_FOREACH(QFileInfo info, dir.entryInfoList(QDir::NoDotAndDotDot | QDir::System | QDir::Hidden  | QDir::AllDirs | QDir::Files, QDir::DirsFirst))
        {
            if(info.isDir())
            {
                result = removeDirectory(info.absoluteFilePath());
            }
            else
            {
                result = QFile::remove(info.absoluteFilePath());
            }

            if(!result)
            {
                return result;
            }
        }
        result = dir.rmdir(dirName);
    }

    return result;
}

/*!
 * \brief Gets module template file names and also aquila file names that need updating.
 * \param[in] directory - directory that will be recursivelly searched for files
 * \return true on success
 */
bool ModuleGenerator::getFileNames(QString directory)
{
    //source directory
    QDir sourceDir(directory);
    if(!sourceDir.exists())
    {
        qCritical("module_generator: %s source directory does not exist", sourceDir.path().toStdString().c_str());
        return false;
    }

    //get file name paths from the directory
    QStringList tmpFiles = sourceDir.entryList(QDir::Files);
    for(int i=0; i<tmpFiles.count(); i++)
    {
        QString srcName = directory + QDir::separator() + tmpFiles[i];
        file.append(srcName);
    }

    //get directory names and get their file paths recursively
    QStringList tmpDirs = sourceDir.entryList(QDir::AllDirs | QDir::NoDotAndDotDot);
    for(int i=0; i<tmpDirs.count(); i++)
    {
        QString srcName = directory + QDir::separator() + tmpDirs[i];
        getFileNames(srcName);
    }

    return true;
}

