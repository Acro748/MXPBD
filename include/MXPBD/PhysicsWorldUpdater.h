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
            auto it = skeletonQueue.find(refr->formID);
            if (it == skeletonQueue.end())
            {
                skeletonQueue.emplace(refr->formID, 2);
            }
		}
		std::mutex facegenQueueLock;
        std::unordered_map<RE::FormID, std::uint8_t> facegenQueue; // formID, delay
		inline void RegisterFacegenQueue(RE::TESObjectREFR* refr) {
            if (!refr)
                return;
            std::lock_guard lg(facegenQueueLock);
            auto it = facegenQueue.find(refr->formID);
            if (it == facegenQueue.end())
            {
                facegenQueue.emplace(refr->formID, 2);
            }
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
            auto it = clothQueue.find(refr->formID);
            if (isAttach) {
                if (it == clothQueue.end())
                {
                    clothQueue.emplace(refr->formID, slotQueue{.bipedBit = 1u << bipedSlot, .delay = 2});
                }
                else
                {
                    it->second.bipedBit |= 1 << bipedSlot;
                }
            } 
            else {
                if (it != clothQueue.end())
                    it->second.bipedBit &= ~(1 << bipedSlot);
            }
        }
	protected:
		void onEvent(const Mus::FrameEvent& e) override;
        void onEvent(const Mus::FacegenNiNodeEvent& e) override;
        void onEvent(const Mus::ArmorAttachEvent& e) override;
        void onEvent(const Mus::PlayerCellChangeEvent& e) override;
	};
}
