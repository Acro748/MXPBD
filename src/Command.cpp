#include "Command.h"

namespace Mus {
	bool Execute(const RE::SCRIPT_PARAMETER* a_paramInfo, RE::SCRIPT_FUNCTION::ScriptData* a_scriptData, RE::TESObjectREFR* a_thisObj, RE::TESObjectREFR* a_containingObj,
		RE::Script* a_scriptObj, RE::ScriptLocals* a_locals, double& a_result, std::uint32_t& a_opcodeOffsetPtr)
	{
		char buffer1[MAX_PATH];
		memset(buffer1, 0, MAX_PATH);
		char buffer2[MAX_PATH];
		memset(buffer2, 0, MAX_PATH);

		if (!RE::Script::ParseParameters(a_paramInfo, a_scriptData, a_opcodeOffsetPtr, a_thisObj, a_containingObj, a_scriptObj, a_locals,
			buffer1, buffer2))
		{
			return false;
		}

		auto Console = RE::ConsoleLog::GetSingleton();
		if (!Console)
			return false;

		RE::Actor* a_actor = RE::PlayerCharacter::GetSingleton();
		if (a_thisObj && a_thisObj->Is(RE::FormType::ActorCharacter))
			a_actor = skyrim_cast<RE::Actor*>(a_thisObj);
		if (!a_actor)
			return false;

		const std::string str1 = RE::BSFixedString(buffer1).c_str();
		const std::string str2 = RE::BSFixedString(buffer2).c_str();

		if (IsSameString(str1, "reload"))
		{
            if (IsSameString(str2, "config"))
            {
                Config::GetSingleton().LoadConfig();
            }
            else
            {
                ConditionManager::GetSingleton().InitialCondition();
                Config::GetSingleton().LoadSkeletonFile();
                ConditionManager::GetSingleton().SortConditions();
                Config::GetSingleton().LoadSMPDefaultConfig();
                if (a_actor->loadedData && a_actor->loadedData->data3D)
                    MXPBD::XPBDWorldSystem::GetSingleton().ReloadPhysics(a_actor);
            }
		}
		else if (IsSameString(str1, "reset"))
		{
            if (a_actor->loadedData && a_actor->loadedData->data3D)
                MXPBD::XPBDWorldSystem::GetSingleton().Reset(a_actor);
		}

		return false;
	}

	void Commandhook()
    {
        logger::info("Command Console Hooking...");
        RE::SCRIPT_FUNCTION* command = RE::SCRIPT_FUNCTION::LocateConsoleCommand("WaterColor");
        if (command)
        {
            static RE::SCRIPT_PARAMETER params[2];
            params[0].paramType = RE::SCRIPT_PARAM_TYPE::kChar;
            params[0].paramName = "String (optional)";
            params[0].optional = true;
            params[1].paramType = RE::SCRIPT_PARAM_TYPE::kChar;
            params[1].paramName = "String (optional)";
            params[1].optional = true;

            RE::SCRIPT_FUNCTION cmd = *command;
            cmd.functionName = "MXPBD";
            cmd.shortName = "MXPBD";
            cmd.helpString = "MXPBD";
            cmd.referenceFunction = false;
            cmd.numParams = 2;
            cmd.params = params;
            cmd.executeFunction = Execute;
            cmd.editorFilter = 0;
            REL::safe_write(reinterpret_cast<uintptr_t>(command), &cmd, sizeof(cmd));
        }
        else
            logger::error("Failed to Get Console Command");
    }
}