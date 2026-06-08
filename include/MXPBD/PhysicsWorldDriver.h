#pragma once

namespace MXPBD {
    class XPBDDriver {
    public:
        void AddDriver(RE::TESObjectREFR* object, const Internal::PhysicsWorldContext& context, const DriverInput& input);
        void RemoveDriver(const Internal::PhysicsWorldContext& context, const Internal::RemoveDataList& removeList);

        void RunDriver(Internal::PhysicsWorldContext& context, const float deltaTime);
        void UpdateNewBoneIndex(TBB_ThreadPool* threadPool, const std::vector<std::uint32_t>& oldToNewBoneIdx);
        void ReorderMaps(TBB_ThreadPool* threadPool, const Internal::PhysicsWorldContext& context);
    private:
        bool orderDirty = false;

        Driver::DirectionPosOffsets directionPosOffsets;
        std::vector<std::uint32_t> directionPosOffsetsOrder;

        Driver::DirectionRotOffsets directionRotOffsets;
        std::vector<std::uint32_t> directionRotOffsetsOrder;

        inline bool IsDisable(const Internal::PhysicsWorldContext& context, const std::uint32_t objIdx) const {
            return context.objectDatas->isDisable.size() <= objIdx || context.objectDatas->isDisable[objIdx];
        };
        void LoggingTimeProfiler(const std::string& funcName, const double ms) const;
    };
}
