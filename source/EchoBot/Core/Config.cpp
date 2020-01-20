#include "Config.h"
#include "EchoBot/Utilities/Utilities.h"

#include <fstream>
#include <dlfcn.h>

namespace echobot{
    namespace Config{
        namespace {
            // Initialize global variables, put in anonymous namespace to hide them
            bool mConfigurationLoaded = false;
            std::string mConfigFilename = "";
            std::string mBasePath = "";
            std::string mDataPath;
            std::string mConfigPath;
            std::string mTestDataPath;
            std::string mLibraryPath;
        }


        std::string getPath() {
            if (mBasePath != "")
                return mBasePath;
            std::string path = "";
            std::string slash = "/";

            Dl_info dl_info;
            int ret = dladdr((void *)&getPath, &dl_info);
            const char* dlpath = dl_info.dli_fname;
            path = std::string(dlpath);
            // Remove lib name and lib folder
            int libPos = path.rfind(slash + "lib" + slash);

            path = path.substr(0, libPos);
            path = path + slash; // Make sure there is a slash at the end

            return path;
        }

        void loadConfiguration() {
            if (mConfigurationLoaded)
                return;

            // Set default paths
            mDataPath = getPath() + "../data/";
            mLibraryPath = getPath() + "/lib/";
            mConfigPath = getPath() + "../config/";

            // Read and parse configuration file
            // It should reside in the build folder when compiling, and in the root folder when using release
            std::string filename;
            if(mConfigFilename == "") {
                filename = getPath() + "echobot_configuration.txt";
            }
            else {
                filename = mConfigFilename;
            }

            std::ifstream file(filename);
            if (!file.is_open()) {
                mConfigurationLoaded = true;
                return;
            }

            std::string line;
            std::getline(file, line);
            while (!file.eof()) {
                trim(line);
                if (line[0] == '#' || line.size() == 0) {
                    // Comment or empty line, skip
                    std::getline(file, line);
                    continue;
                }
                std::vector<std::string> list = split(line, "=");
                std::string key = list[0];
                std::string value = list[1];
                trim(key);
                trim(value);
                value = replace(value, "@ROOT@", getPath());

                if (key == "DataPath") {
                    mDataPath = value;
                }

                std::getline(file, line);
            }
            file.close();
            mConfigurationLoaded = true;
        }

        std::string getRegistrationDataPath() {
            loadConfiguration();
            return mDataPath + "registration/";
        }

        std::string getDataPath() {
            loadConfiguration();
            return mDataPath;
        }

        std::string getTestDataPath() {
            loadConfiguration();
            return mDataPath + "testdata/";
        }

        std::string getConfigPath() {
            loadConfiguration();
            return mConfigPath;
        }
    }
}

