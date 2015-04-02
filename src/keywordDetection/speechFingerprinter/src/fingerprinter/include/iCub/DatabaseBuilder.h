/*
 * DatabaseBuilder.h
 *
 *  Created on: Jun 26, 2012
 *      Author: Christian Dondrup
 */

#ifndef DATABASEBUILDER_H_
#define DATABASEBUILDER_H_

#include <string>
#include <dirent.h>
#include <iostream>
#include <vector>
#include <cstring>
#include <fstream>

#include <musicretr-1.0/keypointdb.h>

#include "iCub/Constants.h"

using namespace std;

/**
 * This class builds a keys and files database from the encoded keys files.
 */
class DatabaseBuilder {
private:
	string workdir;

	/**
	 * @brief Creates the filelist.txt file in the working directory.
	 * @param workdir The directory where the keys files are stored and the filelist.txt will be stored.
	 */
	bool createFileList(string workdir);

public:
	/**
	 * @brief Creates a new DatabaseBuilder object which works on the given working directory.
	 * @param workdir The directory where the keys files are located
	 */
	DatabaseBuilder(string workdir)
		:workdir(workdir) {}

	/**
	 * @brief Creates the db.fdb and db.kdb files in the working directoy.
	 */
	bool create();
};




#endif /* DATABASEBUILDER_H_ */
