#include "hook.h"
#include "detours/detours.h"
#include <xbyak/xbyak.h>

namespace Mus {
	constexpr REL::VariantID GameLoopFunction(35565, 36564, 0x005BAB10);
    constexpr REL::VariantID BSFaceGenNiNodeFunction(26406, 26987, 0x003E81B0);
	constexpr REL::VariantID ArmorAttachFunction(15535, 15712, 0x001DB9E0);
    constexpr REL::VariantID ArmorDetachFunction(37945, 38901, 0x006411A0);

	EventDispatcherImpl<FrameEvent>  g_frameEventDispatcher;
	EventDispatcherImpl<QuitGameEvent>  g_quitGameEventDispatcher;
	EventDispatcherImpl<FacegenNiNodeEvent> g_facegenNiNodeEventDispatcher;
	EventDispatcherImpl<ArmorAttachEvent> g_armorAttachEventDispatcher;
	EventDispatcherImpl<ArmorDetachEvent> g_armorDetachEventDispatcher;
    EventDispatcherImpl<Load3DEvent> g_load3DEventDispatcher;
	EventDispatcherImpl<PlayerCellChangeEvent> g_playerCellChangeEventDispatcher;

	typedef void (*_onFaceGen)(RE::BSFaceGenNiNode*, RE::NiNode*, RE::BSGeometry*, RE::BSTriShape*);
	REL::Relocation<_onFaceGen> onFaceGen_Orig(BSFaceGenNiNodeFunction);
	void __fastcall onFaceGen(RE::BSFaceGenNiNode* facegen, RE::NiNode* root, RE::BSGeometry* geometry, RE::BSTriShape* triShape)
	{
		FacegenNiNodeEvent e;
		e.root = root;
		e.facegenNiNode = facegen;
        e.geometry = geometry;
        g_facegenNiNodeEventDispatcher.dispatch(e);
	}

	typedef RE::NiAVObject* (*_ArmorAttachFunction)(void*, RE::NiNode*, RE::NiNode*, std::int32_t, void*, void*, void*, void*, char, std::int32_t, void*);
	REL::Relocation<_ArmorAttachFunction> onArmorAttachFunction_Orig(ArmorAttachFunction);
	RE::NiAVObject* __fastcall onArmorAttachFunction(void* unk1, RE::NiNode* armor, RE::NiNode* skeleton, std::int32_t bipedSlot, void* unk4, void* unk5, void* unk6, void* unk7, char unk8, int unk9, void* unk10)
	{
		ArmorAttachEvent e;
		if (skeleton)
			e.actor = skyrim_cast<RE::Actor*>(skeleton->GetUserData());
		e.armor = armor;
		e.skeleton = skeleton;
		e.bipedSlot = bipedSlot;
		g_armorAttachEventDispatcher.dispatch(e);

		auto result = onArmorAttachFunction_Orig(unk1, armor, skeleton, bipedSlot, unk4, unk5, unk6, unk7, unk8, unk9, unk10);
		
		if (result)
        {
            e.attachedNode = result;
			e.hasAttached = true;
		}

		g_armorAttachEventDispatcher.dispatch(e);
        return result;
	}

	typedef bool (*_ArmorDetachFunction)(void*, RE::Actor*, RE::TESForm*, RE::BaseExtraList*, std::int32_t, RE::BGSEquipSlot*, bool, bool, bool, bool, void*);
    REL::Relocation<_ArmorDetachFunction> onArmorDetachFunction_Orig(ArmorDetachFunction);
	bool __fastcall onArmorDetachFunction(void* unk1, RE::Actor* actor, RE::TESForm* item, RE::BaseExtraList* extraData, std::int32_t count, RE::BGSEquipSlot* equipSlot, bool unkFlag7, bool preventEquip, bool unkFlag9, bool unkFlag10, void* unk11)
	{
        ArmorDetachEvent e;
        e.actor = actor;
        e.hasDetached = false;
        g_armorDetachEventDispatcher.dispatch(e);

        bool result = onArmorDetachFunction_Orig(unk1, actor, item, extraData, count, equipSlot, unkFlag7, preventEquip, unkFlag9, unkFlag10, unk11);

        e.hasDetached = true;
        g_armorDetachEventDispatcher.dispatch(e);
        return result;
	}

	void hookFacegen()
	{
		DetourAttach(&(PVOID&)onFaceGen_Orig, onFaceGen);
	}
	void hookArmorAttach()
	{
		DetourAttach(&(PVOID&)onArmorAttachFunction_Orig, onArmorAttachFunction);
	}
	void hookArmorDetach()
	{
		DetourAttach(&(PVOID&)onArmorAttachFunction_Orig, onArmorDetachFunction);
	}

	RE::FormID PlayerCurrentCell = 0;
	bool IsPlayerExterior = false;
	typedef void (*_NullSub)();
	REL::Relocation<_NullSub> NullSubOrig;
	void onNullSub()
	{
		NullSubOrig();

		auto main = RE::Main::GetSingleton();
		if (main->quitGame)
		{
			QuitGameEvent e;
			g_quitGameEventDispatcher.dispatch(e);
		}
		else
		{
			auto p = RE::PlayerCharacter::GetSingleton();
			if (!p)
				return;
			if (auto currentCell = p->GetParentCell(); currentCell)
			{
				if (PlayerCurrentCell != 0)
				{
					if (PlayerCurrentCell != currentCell->formID)
					{
						PlayerCellChangeEvent ce;
						ce.IsExterior = currentCell->IsExteriorCell();
						ce.IsChangedInOut = IsPlayerExterior != ce.IsExterior;
						g_playerCellChangeEventDispatcher.dispatch(ce);
					}
				}
				PlayerCurrentCell = currentCell->formID;
				IsPlayerExterior = currentCell->IsExteriorCell();
			}

			FrameEvent e;
			e.gamePaused = main ? main->freezeTime : false;
			const auto menu = RE::UI::GetSingleton();
			IsGamePaused.store(((e.gamePaused || (menu && menu->numPausesGame > 0)) && !IsRaceSexMenu.load()) || IsMainMenu.load());
			//currentTime = std::clock();
			g_frameEventDispatcher.dispatch(e);
		}
	}
	void hookEngineTrampoline(SKSE::Trampoline& trampoline)
	{
		//NullSub_594, NullSub_471, NullSub_611
		constexpr auto GameLoopFunctionOffset = REL::VariantOffset(0x748, 0xC26, 0x7EE);
		NullSubOrig = trampoline.write_call<5>(GameLoopFunction.address() + GameLoopFunctionOffset.offset(), onNullSub);
	}

    typedef RE::NiAVObject* (*_Load3D)(RE::TESObjectREFR*, bool);
    _Load3D Load3DOrig = nullptr;
	RE::NiAVObject* __fastcall onLoad3D(RE::TESObjectREFR* reference, bool backgroundLoad)
	{
		auto result = Load3DOrig(reference, backgroundLoad);
		if (result)
		{
			Load3DEvent e;
			e.reference = reference;
			e.loadedObject = result;
			g_load3DEventDispatcher.dispatch(e);
		}
		return result;
    }
	void hookLoad3D()
	{
        REL::Relocation<std::uintptr_t> vtbl(RE::VTABLE_TESObjectREFR[0]);
        Load3DOrig = reinterpret_cast<_Load3D>(vtbl.write_vfunc(0x6A, onLoad3D));
    }

	void fixFaceGenBoneLimit()
    {
        constexpr REL::VariantID BSFaceGenSingleNiNodeFunction(26406, 26987, 0x003E81B0);
        REL::safe_write<std::uint8_t>(BSFaceGenSingleNiNodeFunction.address() + 0x96, 0x7);

        constexpr REL::VariantID HeadPartBoneLimit(24330, 24836, 0x0037AE28);
        constexpr REL::VariantOffset HeadPartBoneLimitOffset(0x58, 0x75, 0x00);
        std::uintptr_t BoneLimit = HeadPartBoneLimit.address() + HeadPartBoneLimitOffset.offset();

        auto& trampoline = SKSE::GetTrampoline();
        SKSE::AllocTrampoline(41);
        struct FaceGenBoneLimitPatchSE : Xbyak::CodeGenerator
        {
            FaceGenBoneLimitPatchSE(std::uintptr_t a_jumpAddr)
            {
                Xbyak::Label jumpLabel;

                mov(esi, ptr[rax + 0x58]);
                cmp(esi, 9);
                jl(jumpLabel);
                mov(esi, 8);

                L(jumpLabel);
                jmp(ptr[rip]);
                dq(a_jumpAddr + 0x7);
            }
        };
        struct FaceGenBoneLimitPatchAE : Xbyak::CodeGenerator
        {
            FaceGenBoneLimitPatchAE(std::uintptr_t a_jumpAddr)
            {
                Xbyak::Label jumpLabel;

                mov(ebp, ptr[rax + 0x58]);
                cmp(ebp, 9);
                jl(jumpLabel);
                mov(ebp, 8);

                L(jumpLabel);
                jmp(ptr[rip]);
                dq(a_jumpAddr + 0x7);
            }
        };
        if (REL::Module::IsSE() || REL::Module::IsVR())
        {
            FaceGenBoneLimitPatchSE faceGenBoneLimitPatch(BoneLimit);
            faceGenBoneLimitPatch.ready();
            trampoline.write_branch<5>(BoneLimit, trampoline.allocate(faceGenBoneLimitPatch));
        }
        else
        {
            FaceGenBoneLimitPatchAE faceGenBoneLimitPatch(BoneLimit);
            faceGenBoneLimitPatch.ready();
            trampoline.write_branch<5>(BoneLimit, trampoline.allocate(faceGenBoneLimitPatch));
        }
        return;
    }

	void hook()
	{
		logger::info("Skyrim Hooking...");
		DetourRestoreAfterWith();
		DetourTransactionBegin();
		DetourUpdateThread(GetCurrentThread());
		hookFacegen();
		hookArmorAttach();
        hookArmorDetach();
		DetourTransactionCommit();

		auto& trampoline = SKSE::GetTrampoline();
        SKSE::AllocTrampoline(14);
		hookEngineTrampoline(trampoline);

		//hookLoad3D();
	}
}
