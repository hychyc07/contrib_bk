/*
 * DatabaseBuilder.cpp
 *
 *  Created on: Jun 26, 2012
 *      Author: Christian Dondrup
 */

#include "iCub/DatabaseBuilder.h"

using namespace std;

bool DatabaseBuilder::create() {
	//Creating the filelist.txt file
	if (!createFileList(workdir))
		return false;

	//Creating the db.fdb and db.kdb files in the working directory
	KeypointDB kdb((char*) string(workdir + "/" + FILE_LIST).c_str(),
			(char*) string(workdir + "/" + FILES_DB).c_str(),
			(char*) string(workdir + "/" + KEYS_DB).c_str());

	//Return true if everything went well
	return true;
}

bool DatabaseBuilder::createFileList(string workdir) {
	/* Reading .keys files from working directory */
	//Opening the working directory
	DIR *dp;
	struct dirent *dirp;
	if ((dp = opendir(workdir.c_str())) == NULL) {
		cout << "ERROR: Could not open " << workdir << endl;
		return false;
	}
	//Creating a vector of file names for all .keys files
	vector<string> files;
	while ((dirp = readdir(dp)) != NULL) {
		if (string(dirp->d_name).substr(
				string(dirp->d_name).find_last_of(".") + 1) == "keys")
			files.push_back(workdir + "/" + string(dirp->d_name));
	}
	//Closing directory
	closedir(dp);

	/* Creating the filelist.txt file */
	//Opening the workdir/filelist.txt file for writing
	ofstream outfile;
	outfile.open(string(workdir + "/" + FILE_LIST).c_str(), ofstream::out);
	if (!outfile) {
		cout << "ERROR: Could write to " << workdir << "/" << FILE_LIST << endl;
		return false;
	}
	//Writing the file names to the filelist.txt
	for (int i = 0; i < files.size(); i++) {
		if (outfile.good()) {
			outfile.write(string(files.at(i) + "\n").c_str(),
					files.at(i).length() + 1);
		} else {
			cout << "ERROR: Could write to " << workdir << "/" << FILE_LIST
			<< endl;
			return false;
		}
	}
	//Closing the filelist.txt file
	outfile.close();

	//Return true if everything went well
	return true;
}

