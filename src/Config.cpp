#include "Config.h"
#include "tinyxml2.h"

namespace Mus {
    bool Config::LoadLogging()
    {
        std::string configPath = GetRuntimeSKSEDirectory();
        configPath += SKSE::PluginDeclaration::GetSingleton()->GetName().data();
        configPath += ".ini";

        std::ifstream file(configPath);

        if (!file.is_open())
        {
            std::transform(configPath.begin(), configPath.end(), configPath.begin(), ::tolower);
            file.open(configPath);
        }

        if (!file.is_open())
        {
            return false;
        }

        std::string line;
        std::string currentSetting;
        while (std::getline(file, line))
        {
            //trim(line);
            skipComments(line);
            trim(line);
            if (line.length() == 0)
                continue;

            if (line.substr(0, 1) == "[")
            {
                currentSetting = line;
                continue;
            }
            std::string variableName;
            std::string variableValue = GetConfigSetting(line, variableName);
            if (currentSetting == "[Debug]")
            {
                if (variableName == "logLevel")
                {
                    logLevel = spdlog::level::from_str(variableValue);
                }
                else if (variableName == "flushLevel")
                {
                    flushLevel = spdlog::level::from_str(variableValue);
                }
            }
        }
        return true;
    }

    bool Config::LoadConfig() {
        std::string configPath = GetRuntimeSKSEDirectory();
        configPath += SKSE::PluginDeclaration::GetSingleton()->GetName().data();
        configPath += ".ini";

        std::ifstream file(configPath);

        if (!file.is_open())
        {
			lowLetter(configPath);

            file.open(configPath);
            if (!file.is_open())
            {
                logger::critical("Unable to load Config file.");
                return false;
            }
        }

        return LoadConfig(file);
    }

    bool Config::LoadConfig(std::ifstream& configfile)
    {
        std::string line;
        std::string currentSetting;
        while (std::getline(configfile, line))
        {
            //trim(line);
            skipComments(line);
            trim(line);
            if (line.length() == 0)
                continue;

            if (line.substr(0, 1) == "[")
            {
                currentSetting = line;
                continue;
            }
            std::string variableName;
            std::string variableValue = GetConfigSetting(line, variableName);
            if (currentSetting == "[Debug]")
            {
                if (variableName == "logLevel")
                {
                    logLevel = spdlog::level::from_str(variableValue);
                }
                else if (variableName == "flushLevel")
                {
                    flushLevel = spdlog::level::from_str(variableValue);
                }
            }
            else if (currentSetting == "[General]")
            {
                if (variableName == "IterationMax")
                {
                    IterationMax = std::max(1u, GetUIntValue(variableValue));
                }
                else if (variableName == "SmallGridSize")
                {
                    SmallGridSize = GetFloatValue(variableValue);
                }
                else if (variableName == "LargeGridSize")
                {
                    LargeGridSize = GetFloatValue(variableValue);
                }
                else if (variableName == "RotationClampSpeed")
                {
                    RotationClampSpeed = GetFloatValue(variableValue);
                }
                else if (variableName == "CollisionConvergence")
                {
                    CollisionConvergence = GetFloatValue(variableValue);
                }
                else if (variableName == "GroundDetectRange")
                {
                    GroundDetectRange = GetFloatValue(variableValue);
                }
                else if (variableName == "GroundDetectQuality")
                {
                    GroundDetectQuality = GetUIntValue(variableValue);
                }
                else if (variableName == "ValidBoneWeightThreshold")
                {
                    ValidBoneWeightThreshold = GetFloatValue(variableValue);
                }
			}
        }
        return true;
    }

    bool MultipleConfig::LoadSkeletonFile()
    {
        std::string conditionPath = GetRuntimeSKSEDirectory();
        conditionPath += SKSE::PluginDeclaration::GetSingleton()->GetName().data();
        auto files = GetAllFiles(conditionPath);
        tbb::parallel_for(
            tbb::blocked_range<std::size_t>(0, files.size()),
            [&](const tbb::blocked_range<std::size_t>& r) {
                for (std::size_t i = r.begin(); i != r.end(); ++i)
                {
                    std::u8string filename_utf8 = files[i].filename().u8string();
                    std::string filename(filename_utf8.begin(), filename_utf8.end());
                    if (filename == "." || filename == "..")
                        return;
                    if (!stringEndsWith(filename, ".xml"))
                        return;

                    std::string filePath = conditionPath + "\\" + filename;
                    tinyxml2::XMLDocument doc;
                    const auto error = doc.LoadFile(filePath.c_str());
                    switch (error)
                    {
                    case tinyxml2::XML_SUCCESS:
                        break;
                    case tinyxml2::XML_ERROR_FILE_NOT_FOUND:
                    case tinyxml2::XML_ERROR_FILE_COULD_NOT_BE_OPENED:
                    case tinyxml2::XML_ERROR_FILE_READ_ERROR:
                        logger::error("{} : Unable to open the file ({})", filePath, std::to_underlying(error));
                        return;
                        break;
                    default:
                        logger::error("{} : The file's xml format is invalid ({})", filePath, std::to_underlying(error));
                        return;
                        break;
                    };

                    tinyxml2::XMLElement* root = doc.RootElement();
                    logger::info("File found: {}", filename);
                    ConditionManager::Condition condition;
                    condition.fileName = filename;

                    tinyxml2::XMLElement* ConditionRoot = root->FirstChildElement("Condition");
                    if (!ConditionRoot)
                        return;
                    const char* orgCondition = ConditionRoot->Attribute("condition");
                    if (!orgCondition)
                        return;
                    logger::info("{} : condition {}", filename, orgCondition);
                    condition.originalCondition = orgCondition;
                    ConditionRoot->QueryIntAttribute("priority", &condition.Priority);
                    logger::info("{} : priority {}", filename, condition.Priority);

                    if (!MXPBD::GetPhysicsInput(root, filename, condition.setting))
                        return;
                    ConditionManager::GetSingleton().RegisterCondition(condition);
                }
            },
            tbb::auto_partitioner()
        );
        return true;
    }
}
