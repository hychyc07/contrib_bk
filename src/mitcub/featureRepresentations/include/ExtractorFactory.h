




#ifndef __EXTRACTOR_FACTORY__
#define __EXTRACTOR_FACTORY__


#include <string>
#include <cassert>
#include <stdexcept>
#include <sstream>

#include <map>
#include <deque>

#include "Extractor.h"
#include "ExtractorUtils.h"

class ExtractorFactory {
protected:
    /**
     * The map that stores the key to object mapping.
     */
    std::map<std::string,Extractor*> map;

private:

    /**
     * Constructor (empty).
     */
    ExtractorFactory() { }

    /**
     * Copy Constructor (private and unimplemented on purpose).
     */
    ExtractorFactory(const ExtractorFactory& other);

    /**
     * Assignment operator (private and unimplemented on purpose).
     */
    ExtractorFactory& operator=(const ExtractorFactory& other);

    /**
     * The default destructor of the Factory template. This destructor takes
     * responsibility for deleting the prototype objects that have been
     * registered during its lifetime.
     */
    virtual ~ExtractorFactory() {
        for(std::map<std::string,Extractor*>::iterator it = this->map.begin(); it != this->map.end(); it++) {
            delete it->second;
        }
    }

public:
    /**
     * An instance retrieval method that follows the Singleton pattern.
     *
     * Note that this implementation should not be considered thread safe and
     * that problems may arise. However, due to the nature of the expected use
     * this will not be likely to result in problems.
     *
     * See http://www.oaklib.org/docs/oak/singleton.html for more information.
     *
     * @return the singleton factory instance
     */
    static ExtractorFactory &instance() {
        static ExtractorFactory instance;
        return instance;
    }

    /**
     * Registers a prototype object that can be used to create clones. Strictly
     * speaking, for this application the object only has to be able to return a
     * new object of its own type, regardless of the internal state of that
     * object.
     *
     * @param prototype the prototype object
     * @throw runtime error if the name of the prototype is empty
     * @throw runtime error if another object is registered with the same name
     */
    void registerExtractor(const string &type, Extractor *e)
    {
        if(type == "") {
            throw std::runtime_error("Cannot register Extractor with empty type; please specify a unique key.");
        }

        if(this->map.count(type) > 0) {
            std::ostringstream buffer;
            buffer << "Extractor of type '" << type
                   << "' has already been registered; please specify a unique key.";
            throw std::runtime_error(buffer.str());
        }

        this->map[type] = e;
    }


    /**
     * Creates a new object given a specific type of key. The receiving end
     * takes ownership of the returned pointer.
     *
     * @param key a key that identifies the type of machine
     * @return a copied object of the specified type
     * @throw runtime error if no object has been registered with the same key
     */
    Extractor* create(ResourceFinder &rf, const string name, int state=STATE_EXTRACT) const
    {
        string context_path;
        string save_path;
        string set_state;
        string data_path="";

        switch(state)
        {
            case(STATE_EXTRACT):
            {
                context_path=rf.findGroup("general").find("context_path_extractors").asString().c_str();
                save_path=rf.findGroup("general").find("save_path_extractors").asString().c_str();
                set_state="extract";
                break;
            }

            case(STATE_DICTIONARIZE):
            {
                context_path=rf.findGroup("general").find("context_path_dictionarizers").asString().c_str();
                save_path=rf.findGroup("general").find("save_path_dictionarizers").asString().c_str();
                set_state="dictionarize";
                break;
            }
        }


        ResourceFinder ext_rf;
        ext_rf.setVerbose(rf.check("verbose") || rf.findGroup("general").check("verbose"));
        initResourceFinder(&ext_rf,context_path,name);

        ext_rf.setDefault("name",name.c_str());

        string type=ext_rf.find("type").asString().c_str();

        //check if there exists the type associated to the extractor
        if(type=="" || this->map.count(type)==0)
            return NULL;


        if(context_path.size()>0)
            data_path=context_path+"/";
        data_path+=name;


        ext_rf.setDefault("state",set_state.c_str());
        ext_rf.setDefault("data_path",data_path.c_str());
        ext_rf.setDefault("save_path",save_path.c_str());

        return this->map.find(type)->second->create(ext_rf);
    }
};

#endif



