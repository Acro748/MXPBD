#include "Condition.h"

namespace Mus {
	void ConditionManager::InitialConditionMap()
	{
		ConditionMap.clear();
		auto types = magic_enum::enum_entries<ConditionType>();
		for (auto& type : types)
		{
			ConditionMap.emplace(lowLetter(type.second.data()), type.first);
		}
	}

	bool ConditionManager::ConditionCheck(RE::Actor* a_actor, const ConditionPtr& condition) const
	{
        if (!a_actor)
            return false;
        return std::ranges::all_of(condition->AND, [&](const auto& AND) {
            return std::ranges::any_of(AND, [&](const auto& OR) {
                bool isTrue = OR.NOT ? !OR.conditionFunction->Condition(a_actor) : OR.conditionFunction->Condition(a_actor);
                if (Config::GetSingleton().GetLogLevel() <= spdlog::level::level_enum::debug)
                    Logging(a_actor, OR, isTrue);
                return isTrue;
            });
        });
	}

	MXPBD::PhysicsInput ConditionManager::GetCondition(RE::Actor* a_actor) const
	{
        MXPBD::PhysicsInput settings;
        if (!a_actor)
            return settings;
        for (const auto& condition : ConditionList)
        {
			if (ConditionCheck(a_actor, condition))
			{
                settings.bones.insert_range(condition->setting.bones);
                settings.constraints.insert_range(condition->setting.constraints);
                settings.angularConstraints.insert_range(condition->setting.angularConstraints);
                for (const auto& noColBones : condition->setting.convexHullColliders.noCollideBones)
                {
                    settings.convexHullColliders.noCollideBones[noColBones.first].insert(noColBones.second.begin(), noColBones.second.end());
                }
			}
		}
        return settings;
	}

	bool ConditionManager::RegisterCondition(Condition& condition, bool preventDuplicate)
	{
        if (!ParseConditions(condition))
        {
            logger::error("{} : Invalid condition. so skip.", condition.fileName);
            return false;
        }
        std::lock_guard lg(ConditionListLock);
        if (preventDuplicate) [[unlikely]]
        {
            auto found = std::find_if(ConditionList.begin(), ConditionList.end(), [&](ConditionPtr& a_condition) {
                return a_condition->fileName == condition.fileName;
            });
            if (found != ConditionList.end())
            {
                (*found) = std::make_shared<Condition>(condition);
                logger::info("{} : Found the old condition. so overwrite the condition", condition.fileName);
                return true;
            }
        }
        ConditionList.push_back(std::make_shared<Condition>(condition));
		return true;
	}

	void ConditionManager::SortConditions()
	{
        logger::info("sorting conditions...");
        std::ranges::sort(ConditionList, std::ranges::greater(), [](const ConditionPtr& c) { return c->Priority; });
	}

	bool ConditionManager::ParseConditions(Condition& condition) const
	{
		std::vector<std::string> splittedANDs = split(condition.originalCondition, "AND");

        bool isAllInvalid = true;
		bool firstAND = true;
		for (auto& strAnd : splittedANDs)
		{
			if (!firstAND)
				logger::debug("AND ...");
			firstAND = false;
			std::vector<std::string> splittedORs = split(strAnd, "OR");
			ConditionItemOr conditionOr;

			bool firstOR = true;
			for (auto& strOr : splittedORs)
			{
				ConditionItem Item;
				if (stringStartsWith(strOr, "NOT"))
				{
					Item.NOT = true;
					strOr.erase(0, 3);
					trim(strOr);
				}

				bool invalid = false;
				if (GetConditionType(strOr, Item) == ConditionType::Error)
					invalid = true;
				else
				{
					if (GetConditionFunction(Item))
					{
						conditionOr.emplace_back(Item);
                        logger::debug("{} : {}{}{} ...", condition.fileName, firstOR ? "" : "OR ", Item.NOT ? "NOT " : "", magic_enum::enum_name(Item.type).data());
					}
					else
						invalid = false;
				}

				if (invalid)
                {
                    logger::error("{} : Found invalid condition, so ignore it : {}{}{}", condition.fileName, firstOR ? "" : "OR ", Item.NOT ? "NOT " : "", strOr);
                }
                else
                    isAllInvalid = false;
				firstOR = false;
			}
			condition.AND.emplace_back(conditionOr);
		}
        return !isAllInvalid;
	}

	ConditionManager::ConditionType ConditionManager::GetConditionType(std::string line, ConditionItem& item) const
	{
		std::vector<std::string> splittedMain = splitMulti(line, "()");
		if (splittedMain.size() == 0)
		{
			item.type = ConditionType::None;
			return ConditionType::None;
		}
		std::string low = lowLetter(splittedMain[0]);
		if (auto found = ConditionMap.find(low); found != ConditionMap.end())
			item.type = found->second;
		else
		{
			item.type = ConditionType::Error;
			return ConditionType::Error;
		}

		if (splittedMain.size() > 1)
		{
			std::vector<std::string> splitted = split(splittedMain[1], "|");
			if (splitted.size() == 1)
			{
				item.id = GetHex(splitted[0]);
				item.arg = splitted[0];
			}
			else if (splitted.size() == 2)
			{
				item.pluginName = splitted[0];
				item.id = GetHex(splitted[1]);
				item.arg = splitted[0];
			}
			else if (splitted.size() == 3)
			{
				item.pluginName = splitted[0];
				item.id = GetHex(splitted[1]);
				item.arg = splitted[2];
			}
		}
		return item.type;
	}

	bool ConditionManager::GetConditionFunction(ConditionItem& item) const
	{
		switch (item.type) {
		case ConditionType::HasKeyword:
			item.conditionFunction = std::make_shared<ConditionFragment::HasKeyword>();
			break;
		case ConditionType::HasKeywordEditorID:
			item.conditionFunction = std::make_shared<ConditionFragment::HasKeywordEditorID>();
			break;
		case ConditionType::IsActor:
			item.conditionFunction = std::make_shared<ConditionFragment::IsActor>();
			break;
		case ConditionType::IsActorBase:
			item.conditionFunction = std::make_shared<ConditionFragment::IsActorBase>();
			break;
		case ConditionType::IsRace:
			item.conditionFunction = std::make_shared<ConditionFragment::IsRace>();
			break;
        case ConditionType::IsInFaction:
            item.conditionFunction = std::make_shared<ConditionFragment::IsInFaction>();
			break;
		case ConditionType::IsFemale:
			item.conditionFunction = std::make_shared<ConditionFragment::IsFemale>();
			break;
		case ConditionType::IsChild:
			item.conditionFunction = std::make_shared<ConditionFragment::IsChild>();
			break;
		case ConditionType::None:
			item.conditionFunction = std::make_shared<ConditionFragment::NoneCondition>();
			break;
		}
		if (!item.conditionFunction)
			return false;
		item.conditionFunction->Initial(item);
		return true;
	}

	static RE::NiStream* NiStream_ctor(RE::NiStream* stream) {
		using func_t = decltype(&NiStream_ctor);
		REL::VariantID offset(68971, 70324, 0x00C9EC40);
		REL::Relocation<func_t> func{ offset };
		return func(stream);
	}
	static void NiStream_dtor(RE::NiStream* stream) {
		using func_t = decltype(&NiStream_dtor);
		REL::VariantID offset(68972, 70325, 0x00C9EEA0);
		REL::Relocation<func_t> func{ offset };
		return func(stream);
	}
	namespace ConditionFragment
	{
		void HasKeyword::Initial(ConditionManager::ConditionItem& item)
		{
			keyword = GetFormByID<RE::BGSKeyword*>(item.id, item.pluginName);
		}
		bool HasKeyword::Condition(RE::Actor* actor)
		{
            if (!actor || !keyword)
				return false;
            RE::TESRace* race = actor->GetRace();
            return (actor->HasKeyword(keyword) || (race ? race->HasKeyword(keyword) : false));
		}

		void HasKeywordEditorID::Initial(ConditionManager::ConditionItem& item)
		{
			keywordEditorID = item.arg;
		}
		bool HasKeywordEditorID::Condition(RE::Actor* actor)
		{
            if (!actor || keywordEditorID.empty())
                return false;
            RE::TESRace* race = actor->GetRace();
            return (actor->HasKeywordString(keywordEditorID.c_str()) || (race ? race->HasKeywordString(keywordEditorID.c_str()) : false));
		}

		void IsActor::Initial(ConditionManager::ConditionItem& item)
		{
			form = GetFormByID(item.id, item.pluginName);
		}
		bool IsActor::Condition(RE::Actor* actor)
		{
            if (!actor || !form)
				return false;
            return actor->formID == form->formID;
		}

		void IsActorBase::Initial(ConditionManager::ConditionItem& item)
		{
			form = GetFormByID(item.id, item.pluginName);
		}
		bool IsActorBase::Condition(RE::Actor* actor)
		{
            if (!actor || !form)
				return false;
            auto npc = actor->GetActorBase();
            return npc && npc->formID == form->formID;
		}

		void IsRace::Initial(ConditionManager::ConditionItem& item)
		{
			form = GetFormByID(item.id, item.pluginName);
		}
		bool IsRace::Condition(RE::Actor* actor)
		{
            if (!actor || !form)
                return false;
            RE::TESRace* race = actor->GetRace();
			return race && race->formID == form->formID;
		}

		void IsInFaction::Initial(ConditionManager::ConditionItem& item)
		{
            faction = GetFormByID<RE::TESFaction*>(item.id, item.pluginName);
		}
        bool IsInFaction::Condition(RE::Actor* actor)
		{
            if (!actor || !faction)
                return false;
            return actor->IsInFaction(faction);
		}

		void IsFemale::Initial(ConditionManager::ConditionItem& item)
		{
		}
		bool IsFemale::Condition(RE::Actor* actor)
		{
            auto npc = actor->GetActorBase();
            RE::SEX gender = npc ? npc->GetSex() : RE::SEX::kNone;
            return gender == RE::SEX::kFemale;
		}

		void IsChild::Initial(ConditionManager::ConditionItem& item)
		{
		}
		bool IsChild::Condition(RE::Actor* actor)
		{
            if (!actor)
                return false;
            return actor->IsChild();
		}

		void NoneCondition::Initial(ConditionManager::ConditionItem& item)
		{
		}
		bool NoneCondition::Condition(RE::Actor* actor)
		{
			return true;
		}
	}
}
