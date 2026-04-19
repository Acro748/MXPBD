#include "QuickHull.hpp"
#include "MXPBD/PhysicsWorldUpdater.h"
#include "meshoptimizer.h"

namespace MXPBD
{
    void XPBDWorldUpdater::Init()
    {
        Mus::g_frameEventDispatcher.addListener(this);
        Mus::g_facegenNiNodeEventDispatcher.addListener(this);
        Mus::g_armorAttachEventDispatcher.addListener(this);
        Mus::g_playerCellChangeEventDispatcher.addListener(this);
    }

    void XPBDWorldUpdater::AddSkeletonPhysicsToWorld(RE::Actor* a_actor) const
    {
        if (!a_actor || !a_actor->loadedData || !a_actor->loadedData->data3D)
            return;
        logger::info("Adding physics {:x}", a_actor->formID);
        XPBDWorldSystem::GetSingleton().AddPhysics(a_actor, a_actor->loadedData->data3D->AsNode(), XPBDWorld::skeleton, 0);
    }

    void XPBDWorldUpdater::AddFacegenPhysicsToWorld(RE::Actor* a_actor) const
    {
        if (!a_actor || !a_actor->loadedData || !a_actor->loadedData->data3D)
            return;
        logger::info("Adding physics {:x}", a_actor->formID);
        XPBDWorldSystem::GetSingleton().AddPhysics(a_actor, a_actor->loadedData->data3D->AsNode(), XPBDWorld::facegen, 0);
    }

    void XPBDWorldUpdater::AddClothPhysicsToWorld(RE::Actor* a_actor, RE::NiNode* root, std::uint32_t biped) const
    {
        if (!a_actor || !a_actor->loadedData || !a_actor->loadedData->data3D)
            return;
        if (!root)
        {
            logger::error("{:x} {} : there is no root", a_actor->formID, biped);
            return;
        }
        logger::info("Adding physics {:x}", a_actor->formID);
        XPBDWorldSystem::GetSingleton().AddPhysics(a_actor, root, XPBDWorld::cloth, biped);
    }

    void XPBDWorldUpdater::UpdatePhysicsSetting(RE::Actor* a_actor) const
    {
        if (!a_actor)
            return;
        XPBDWorldSystem::GetSingleton().UpdatePhysicsSetting(a_actor, Mus::ConditionManager::GetSingleton().GetCondition(a_actor));
    }

    void XPBDWorldUpdater::RunSkeletonQueue()
    {
        std::vector<RE::FormID> tasks; // actorID, slotBit
        {
            std::lock_guard lg(skeletonQueueLock);
            for (auto it = skeletonQueue.begin(); it != skeletonQueue.end();)
            {
                if (it->second == 0)
                {
                    tasks.push_back(it->first);
                    it = skeletonQueue.erase(it);
                }
                else
                {
                    it->second--;
                    it++;
                }
            }
        }

        for (const auto& t : tasks)
        {
            auto actor = Mus::GetFormByID<RE::Actor*>(t);
            if (!actor)
                continue;
            AddSkeletonPhysicsToWorld(actor);
        }
    }
    void XPBDWorldUpdater::RunFacegenQueue()
    {
        std::vector<RE::FormID> tasks; // actorID, slotBit
        {
            std::lock_guard lg(facegenQueueLock);
            for (auto it = facegenQueue.begin(); it != facegenQueue.end();)
            {
                if (it->second == 0)
                {
                    tasks.push_back(it->first);
                    it = facegenQueue.erase(it);
                }
                else
                {
                    it->second--;
                    it++;
                }
            }
        }

        for (const auto& t : tasks)
        {
            auto actor = Mus::GetFormByID<RE::Actor*>(t);
            if (!actor)
                continue;
            AddFacegenPhysicsToWorld(actor);
        }
    }
    void XPBDWorldUpdater::RunArmorQueue()
    {
        std::unordered_map<RE::FormID, std::uint32_t> tasks; // actorID, slotBit
        {
            std::lock_guard lg(clothQueueLock);
            for (auto it = clothQueue.begin(); it != clothQueue.end();)
            {
                if (it->second.delay == 0)
                {
                    tasks[it->first] = it->second.bipedBit;
                    it = clothQueue.erase(it);
                }
                else
                {
                    it->second.delay--;
                    it++;
                }
            }
        }

        for (const auto& t : tasks)
        {
            auto actor = GetActor(t.first);
            if (!actor || !actor->loadedData || !actor->loadedData->data3D)
                continue;
            const auto bitElements = GetBitElements(t.second);
            for (auto e : bitElements)
            {
                AddClothPhysicsToWorld(actor, actor->loadedData->data3D->AsNode(), e);
            }
        }
    }

    void XPBDWorldUpdater::onEvent(const Mus::FrameEvent& e)
    {
        if (Mus::IsGamePaused.load())
            return;
        RunSkeletonQueue();
        RunFacegenQueue();
        RunArmorQueue();
    }

    void XPBDWorldUpdater::onEvent(const Mus::FacegenNiNodeEvent& e)
    {
        if (!e.root)
            return;
        auto refr = e.root->GetUserData();
        if (!refr)
            return;
        RegisterSkeletonQueue(refr);
        RegisterFacegenQueue(refr);
    }

    void XPBDWorldUpdater::onEvent(const Mus::ArmorAttachEvent& e)
    {
        if (!e.hasAttached)
            return;
        if (!e.actor)
            return;
        logger::debug("ArmorAttachEvent : {:x} {}", e.actor->formID, e.bipedSlot);
        RegisterSkeletonQueue(e.actor);
        RegisterArmorQueue(e.actor, e.bipedSlot, true);
    }

    void XPBDWorldUpdater::onEvent(const Mus::PlayerCellChangeEvent& e)
    {
        if (!e.IsChangedInOut)
            return;
        XPBDWorldSystem::GetSingleton().Reset();
    }
} // namespace MXPBD
