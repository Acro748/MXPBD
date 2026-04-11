#pragma once

namespace MXPBD {
	class XPBDWorldUpdater :
		public Mus::IEventListener<Mus::FrameEvent>,
		public Mus::IEventListener<Mus::FacegenNiNodeEvent>,
		public Mus::IEventListener<Mus::ActorChangeHeadPartEvent>,
		public Mus::IEventListener<Mus::ArmorAttachEvent>,
		public Mus::IEventListener<Mus::PlayerCellChangeEvent> 
	{
	public:
		[[nodiscard]] static XPBDWorldUpdater& GetSingleton() {
			static XPBDWorldUpdater instance;
			return instance;
		}

        void Init();
        void AddSkeletonPhysicsToWorld(RE::Actor* a_actor, RE::NiNode* root) const;
        void AddClothPhysicsToWorld(RE::Actor* a_actor, RE::NiNode* root, std::uint32_t biped) const;
        void UpdatePhysicsSetting(RE::Actor* a_actor) const;

	private:
        void RunSkeletonQueue();
        void RunArmorQueue();

		struct slotQueue {
            std::uint32_t bipedBit;  // formID, RE::BIPED_MODEL::BipedObjectSlot, delay
            std::uint8_t delay = 0;
		};
		std::mutex armorQueueLock;
        std::unordered_map<RE::FormID, slotQueue> armorQueue; // formID, RE::BIPED_MODEL::BipedObjectSlot, delay
		inline void RegisterArmorQueue(RE::TESObjectREFR* refr, std::uint32_t bipedSlot, bool isAttach) {
            if (!refr)
                return;
            std::lock_guard lg(armorQueueLock);
            if (isAttach) {
                armorQueue[refr->formID].bipedBit |= 1 << bipedSlot;
                armorQueue[refr->formID].delay = 3;
            }
            else {
                if (armorQueue.count(refr->formID) != 0)
                    armorQueue[refr->formID].bipedBit &= ~(1 << bipedSlot);
            }
		}
		std::mutex skeletonQueueLock;
        std::unordered_map<RE::FormID, slotQueue> skeletonQueue; // formID, RE::BIPED_MODEL::BipedObjectSlot, delay
		inline void RegisterSkeletonQueue(RE::TESObjectREFR* refr) {
            if (!refr)
                return;
            std::lock_guard lg(skeletonQueueLock);
            if (skeletonQueue[refr->formID].delay == 0)
                skeletonQueue[refr->formID].delay = 2;
		}
	protected:
		void onEvent(const Mus::FrameEvent& e) override;
        void onEvent(const Mus::FacegenNiNodeEvent& e) override;
        void onEvent(const Mus::ActorChangeHeadPartEvent& e) override;
        void onEvent(const Mus::ArmorAttachEvent& e) override;
        void onEvent(const Mus::PlayerCellChangeEvent& e) override;
	};
}
