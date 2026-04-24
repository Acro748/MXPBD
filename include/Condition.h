#pragma once

namespace Mus {
	namespace ConditionFragment
	{
		class ConditionBase;
	}
	class ConditionManager {
	public :
        ConditionManager() { InitialConditionMap(); };
		~ConditionManager() {};

		[[nodiscard]] static ConditionManager& GetSingleton() {
			static ConditionManager instance;
			return instance;
		}

		void InitialConditionMap();
        void InitialCondition() {
            ConditionList.clear();
        };

		enum ConditionType : std::uint32_t {
			HasKeyword,
			HasKeywordEditorID,
			IsActor,
			IsActorBase,
            IsRace,
            IsInFaction,

			IsFemale,
			IsChild,

			None,
			Error
		};

		struct ConditionItem {
			std::shared_ptr<ConditionFragment::ConditionBase> conditionFunction;

			bool NOT = false;
			ConditionType type;
			std::string pluginName = "";
			RE::FormID id = 0;
			std::string arg = "";
		};
		typedef std::vector<ConditionItem> ConditionItemOr;

		struct Condition {
			std::string fileName;
			std::string originalCondition;
			std::vector<ConditionItemOr> AND;
			std::int32_t Priority = 0;

			MXPBD::PhysicsInput setting;

            bool operator<(const Condition& other) const {
                return Priority < other.Priority;
            }
		};
        typedef std::shared_ptr<Condition> ConditionPtr;
        bool RegisterCondition(Condition& condition, bool preventDuplicate = false);
		void SortConditions();

        MXPBD::PhysicsInput GetCondition(RE::Actor* a_actor) const;
        std::size_t ConditionCount() const {
            return ConditionList.size();
        };
	private:
        std::vector<ConditionPtr> ConditionList;
        std::mutex ConditionListLock; // lock for write only

		std::unordered_map<std::string, ConditionType> ConditionMap;

	    bool ParseConditions(Condition& condition) const;
		ConditionType GetConditionType(std::string line, ConditionItem& item) const;

		bool GetConditionFunction(ConditionItem& item) const;
        bool ConditionCheck(RE::Actor* a_actor, const ConditionPtr& condition) const;

		inline std::vector<std::string> splitCondition(const std::string& s, const std::string& delimiter) const {
            std::string str = trim_copy(s);

            std::vector<std::string> result;
            std::int32_t depth = 0;
            std::size_t start = 0;
            std::size_t delim_len = delimiter.length();

            for (std::size_t i = 0; i < str.length();) {
                if (str[i] == '(') {
                    depth++;
                    i++;
                } else if (str[i] == ')') {
                    depth--;
                    i++;
                } else if (depth == 0 && str.compare(i, delim_len, delimiter) == 0) {
                    result.push_back(str.substr(start, i - start));
                    i += delim_len;
                    start = i;
                } else {
                    i++;
                }
            }
            result.push_back(trim_copy(str.substr(start)));
            return result;
        }

		inline void Logging(RE::Actor* a_actor, const ConditionItem& OR, bool isTrue) const {
			std::string typestr = magic_enum::enum_name(ConditionType(OR.type)).data();
			if (IsContainString(typestr, "EditorID")
				|| IsContainString(typestr, "Type"))
			{
                logger::debug("{} {:x} : Condition {}{}({}) is {}", a_actor->GetName(), a_actor->formID,
							  OR.NOT ? "NOT " : "", typestr, OR.arg,
							  isTrue ? "True" : "False");
			}
			else if (OR.type >= ConditionType::IsFemale)
			{
                logger::debug("{} {:x} : Condition {}{}() is {}", a_actor->GetName(), a_actor->formID,
							  OR.NOT ? "NOT " : "", typestr,
							  isTrue ? "True" : "False");
			}
			else
			{
                logger::debug("{} {:x} : Condition {}{}({}{}{:x}) is {}", a_actor->GetName(), a_actor->formID,
							  OR.NOT ? "NOT " : "", typestr, OR.pluginName, OR.pluginName.empty() ? "" : "|", OR.id,
							  isTrue ? "True" : "False");
			}
        }
	};

	namespace ConditionFragment
	{
		class ConditionBase {
		public:
			virtual ~ConditionBase() = default;

			virtual void Initial(ConditionManager::ConditionItem& item) = 0;
			virtual bool Condition(RE::Actor* actor) = 0;
		protected:
		};

		class HasKeyword : public ConditionBase {
		public:
			HasKeyword() = default;
			void Initial(ConditionManager::ConditionItem& item) override;
            bool Condition(RE::Actor* actor) override;
		private:
			RE::BGSKeyword* keyword = nullptr;
		};

		class HasKeywordEditorID : public ConditionBase {
		public:
			HasKeywordEditorID() = default;
			void Initial(ConditionManager::ConditionItem& item) override;
            bool Condition(RE::Actor* actor) override;
		private:
			std::string keywordEditorID = "";
		};

		class IsActor : public ConditionBase {
		public:
			IsActor() = default;
			void Initial(ConditionManager::ConditionItem& item) override;
			bool Condition(RE::Actor* actor) override;
		private:
			RE::TESForm* form = nullptr;
		};

		class IsActorBase : public ConditionBase {
		public:
			IsActorBase() = default;
			void Initial(ConditionManager::ConditionItem& item) override;
			bool Condition(RE::Actor* actor) override;
		private:
			RE::TESForm* form = nullptr;
		};

		class IsRace : public ConditionBase {
		public:
			IsRace() = default;
			void Initial(ConditionManager::ConditionItem& item) override;
            bool Condition(RE::Actor* actor) override;
		private:
			RE::TESForm* form = nullptr;
		};

		class IsInFaction : public ConditionBase {
		public:
            IsInFaction() = default;
			void Initial(ConditionManager::ConditionItem& item) override;
            bool Condition(RE::Actor* actor) override;
		private:
			RE::TESFaction* faction = nullptr;
		};

		class IsFemale : public ConditionBase {
		public:
			IsFemale() = default;
			void Initial(ConditionManager::ConditionItem& item) override;
            bool Condition(RE::Actor* actor) override;
		};

		class IsChild : public ConditionBase {
		public:
			IsChild() = default;
			void Initial(ConditionManager::ConditionItem& item) override;
            bool Condition(RE::Actor* actor) override;
		};

		class NoneCondition : public ConditionBase {
		public:
			NoneCondition() = default;
			void Initial(ConditionManager::ConditionItem& item) override;
            bool Condition(RE::Actor* actor) override;
		};
	}
}
