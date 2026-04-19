#pragma once

namespace MXPBD {
	class XPBDWorldUpdater :
		public Mus::IEventListener<Mus::FrameEvent>,
		public Mus::IEventListener<Mus::FacegenNiNodeEvent>,
		public Mus::IEventListener<Mus::ArmorAttachEvent>,
		public Mus::IEventListener<Mus::PlayerCellChangeEvent> 
	{
	public:
		[[nodiscard]] static XPBDWorldUpdater& GetSingleton() {
			static XPBDWorldUpdater instance;
			return instance;
		}

        void Init();
        void AddSkeletonPhysicsToWorld(RE::Actor* a_actor) const;
        void AddFacegenPhysicsToWorld(RE::Actor* a_actor) const;
        void AddClothPhysicsToWorld(RE::Actor* a_actor, RE::NiNode* root, std::uint32_t biped) const;
        void UpdatePhysicsSetting(RE::Actor* a_actor) const;

	private:
        void RunSkeletonQueue();
        void RunFacegenQueue();
        void RunArmorQueue();

		std::mutex skeletonQueueLock;
        std::unordered_map<RE::FormID, std::uint8_t> skeletonQueue; // formID, RE::BIPED_MODEL::BipedObjectSlot, delay
		inline void RegisterSkeletonQueue(RE::TESObjectREFR* refr) {
            if (!refr)
                return;
            std::lock_guard lg(skeletonQueueLock);
            if (skeletonQueue[refr->formID] == 0)
                skeletonQueue[refr->formID] = 2;
		}
		std::mutex facegenQueueLock;
        std::unordered_map<RE::FormID, std::uint8_t> facegenQueue; // formID, delay
		inline void RegisterFacegenQueue(RE::TESObjectREFR* refr) {
            if (!refr)
                return;
            std::lock_guard lg(facegenQueueLock);
            if (facegenQueue[refr->formID] == 0)
                facegenQueue[refr->formID] = 2;
        }
        struct slotQueue {
            std::uint32_t bipedBit; // formID, RE::BIPED_MODEL::BipedObjectSlot, delay
            std::uint8_t delay = 0;
        };
        std::mutex clothQueueLock;
        std::unordered_map<RE::FormID, slotQueue> clothQueue; // formID, RE::BIPED_MODEL::BipedObjectSlot, delay
        inline void RegisterArmorQueue(RE::TESObjectREFR* refr, std::uint32_t bipedSlot, bool isAttach) {
            if (!refr)
                return;
            std::lock_guard lg(clothQueueLock);
            if (isAttach) {
                clothQueue[refr->formID].bipedBit |= 1 << bipedSlot;
                clothQueue[refr->formID].delay = 3;
            } else {
                if (clothQueue.count(refr->formID) != 0)
                    clothQueue[refr->formID].bipedBit &= ~(1 << bipedSlot);
            }
        }
	protected:
		void onEvent(const Mus::FrameEvent& e) override;
        void onEvent(const Mus::FacegenNiNodeEvent& e) override;
        void onEvent(const Mus::ArmorAttachEvent& e) override;
        void onEvent(const Mus::PlayerCellChangeEvent& e) override;
	};
}
