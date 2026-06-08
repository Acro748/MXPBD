#include "MXPBD/PhysicsWorldDriver.h"

namespace MXPBD
{
    void XPBDDriver::AddDriver(RE::TESObjectREFR* object, const Internal::PhysicsWorldContext& context, const DriverInput& input)
    {
        if (!object || input.directionPosOffsets.empty())
            return;

        if (orderDirty)
            ReorderMaps(context.threadPool, context);

        const RE::FormID objectID = object->formID;
        const auto it = std::find(context.objectDatas->objectID.begin(), context.objectDatas->objectID.end(), objectID);
        if (it == context.objectDatas->objectID.end())
            return;
        const std::uint32_t objIdx = static_cast<std::uint32_t>(std::distance(context.objectDatas->objectID.begin(), it));

        for (const auto& [name, offsets] : input.directionPosOffsets)
        {
            for (const auto& offset : offsets)
            {
                const auto& nameMap = (*context.boneNameToIdx)[objIdx];
                auto srcIt = nameMap.find(offset.srcBoneName);
                auto dstIt = nameMap.find(offset.dstBoneName);
                if (srcIt == nameMap.end() || dstIt == nameMap.end())
                    continue;

                const std::uint32_t srcIdx = srcIt->second;
                const std::uint32_t dstIdx = dstIt->second;

                directionPosOffsets.driverName.push_back(name);
                directionPosOffsets.srcBoneIdx.push_back(srcIdx);
                directionPosOffsets.dstBoneIdx.push_back(dstIdx);
                directionPosOffsets.isWorldOffset.push_back(offset.isWorldOffset ? 1 : 0);
                directionPosOffsets.baseDirection.push_back(DirectX::XMVector3Normalize(ToVector(offset.baseDirection)));
                directionPosOffsets.targetDirection.push_back(DirectX::XMVector3Normalize(ToVector(offset.targetDirection)));
                directionPosOffsets.minOffset.push_back(ToVector(offset.minOffset * context.physicsBones->physicsScale[srcIdx]));
                directionPosOffsets.maxOffset.push_back(ToVector(offset.maxOffset * context.physicsBones->physicsScale[dstIdx]));
                directionPosOffsets.numOffsets++;
                logger::info("Add direction pos driver {:x} {} -> {}", objectID, offset.srcBoneName, offset.dstBoneName);
            }
        }
        ReorderMaps(context.threadPool, context);
    }

    void XPBDDriver::RemoveDriver(const Internal::PhysicsWorldContext& context, const Internal::RemoveDataList& removeList)
    {
        for (std::uint32_t i = 0; i < directionPosOffsets.numOffsets; ++i)
        {
            const std::uint32_t sbi = directionPosOffsets.srcBoneIdx[i];
            const std::uint32_t dbi = directionPosOffsets.dstBoneIdx[i];
            if (sbi == UINT32_MAX || dbi == UINT32_MAX)
                continue;
            Internal::RemoveData srd(context.physicsBones->objIdx[sbi], context.physicsBones->rootIdx[sbi]);
            Internal::RemoveData drd(context.physicsBones->objIdx[dbi], context.physicsBones->rootIdx[dbi]);
            if (removeList.count(srd) != 0 || removeList.count(drd) != 0)
            {
                directionPosOffsets.srcBoneIdx[i] = UINT32_MAX;
                directionPosOffsets.dstBoneIdx[i] = UINT32_MAX;
            }
        }
        orderDirty = true;
    }

    void XPBDDriver::RunDriver(Internal::PhysicsWorldContext& context, const float deltaTime)
    {
        if (orderDirty)
            ReorderMaps(context.threadPool, context);

        if (directionPosOffsets.numOffsets == 0 && directionRotOffsets.numOffsets == 0)
            return;

        static Internal::TimeProfiler timeProfiler(__func__);
        timeProfiler.Start();

        if (directionPosOffsets.numOffsets > 0)
        {
            tbb::parallel_for(
                tbb::blocked_range<std::uint32_t>(0, directionPosOffsets.numOffsets, 32),
                [&](const tbb::blocked_range<std::uint32_t>& r) {
                    for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                    {
                        const std::uint32_t sbi = directionPosOffsets.srcBoneIdx[i];
                        const std::uint32_t dbi = directionPosOffsets.dstBoneIdx[i];
                        const std::uint32_t soi = context.physicsBones->objIdx[sbi];
                        const std::uint32_t doi = context.physicsBones->objIdx[dbi];
                        if (IsDisable(context, doi))
                            continue;

                        Quaternion spaceRot = qZero;
                        if (directionPosOffsets.isWorldOffset[i])
                        {
                            if (context.objectDatas->npcNode[soi] && context.objectDatas->npcNode[soi]->parent)
                                spaceRot = ToQuaternion(context.objectDatas->npcNode[soi]->parent->world.rotate);
                        }
                        else if (const std::uint32_t spbi = context.physicsBones->parentBoneIdx[sbi]; spbi != UINT32_MAX)
                            spaceRot = context.physicsBones->predRot[spbi];
                        const Vector targetWorldDir = DirectX::XMVector3Rotate(directionPosOffsets.targetDirection[i], spaceRot);
                        const Vector localDir = directionPosOffsets.baseDirection[i];
                        const Vector boneWorldDir = DirectX::XMVector3Rotate(localDir, context.physicsBones->predRot[sbi]);
                        const Vector dotResult = DirectX::XMVector3Dot(boneWorldDir, targetWorldDir);

                        Vector factor = DirectX::XMVectorSaturate(dotResult);
                        const Vector factorSq = DirectX::XMVectorMultiply(factor, factor);
                        const Vector smoothFactor = DirectX::XMVectorSubtract(vThree, DirectX::XMVectorMultiply(vTwo, factor));
                        factor = DirectX::XMVectorMultiply(factorSq, smoothFactor);

                        const Vector diffOffset = DirectX::XMVectorSubtract(directionPosOffsets.maxOffset[i], directionPosOffsets.minOffset[i]);
                        const Vector finalOffset = DirectX::XMVectorMultiplyAdd(diffOffset, factor, directionPosOffsets.minOffset[i]);

                        tbb::spin_mutex::scoped_lock sl(context.physicsBonesLock[dbi]);
                        context.physicsBones->dynamicLinearOffset[dbi] = DirectX::XMVectorAdd(context.physicsBones->dynamicLinearOffset[dbi], finalOffset);

                        {
                            const std::uint32_t oi = context.physicsBones->objIdx[dbi];
                            if (context.objectDatas->objectID[oi] == 0x14)
                            {
                                logger::info("Driver {} : factor: {}, offset: {}, finalOffset: {}",
                                    directionPosOffsets.driverName[i],
                                    DirectX::XMVectorGetX(factor),
                                    ToNiPoint(finalOffset),
                                             ToNiPoint(context.physicsBones->dynamicLinearOffset[dbi]));
                            }
                        }
                    }
                },
                tbb::static_partitioner()
            );
        }

        timeProfiler.End([this](const std::string& name, const double ms) {
            this->LoggingTimeProfiler(name, ms);
        });
    }

    void XPBDDriver::UpdateNewBoneIndex(TBB_ThreadPool* threadPool, const std::vector<std::uint32_t>& oldToNewBoneIdx)
    {
        auto func = [&]() {
            tbb::parallel_invoke(
                [&] {
                    for (std::uint32_t dpoi = 0; dpoi < directionPosOffsets.numOffsets; ++dpoi)
                    {
                        const std::uint32_t sbi = directionPosOffsets.srcBoneIdx[dpoi];
                        const std::uint32_t dbi = directionPosOffsets.dstBoneIdx[dpoi];
                        if (sbi != UINT32_MAX)
                            directionPosOffsets.srcBoneIdx[dpoi] = oldToNewBoneIdx[sbi];
                        if (dbi != UINT32_MAX)
                            directionPosOffsets.dstBoneIdx[dpoi] = oldToNewBoneIdx[dbi];
                    }
                },
                [&] {
                    for (std::uint32_t droi = 0; droi < directionRotOffsets.numOffsets; ++droi)
                    {
                        const std::uint32_t sbi = directionRotOffsets.srcBoneIdx[droi];
                        const std::uint32_t dbi = directionRotOffsets.dstBoneIdx[droi];
                        if (sbi != UINT32_MAX)
                            directionRotOffsets.srcBoneIdx[droi] = oldToNewBoneIdx[sbi];
                        if (dbi != UINT32_MAX)
                            directionRotOffsets.dstBoneIdx[droi] = oldToNewBoneIdx[dbi];
                    }
                });
            orderDirty = true;
        };

        if (threadPool)
            threadPool->Execute(func);
        else
            func();
    }

    void XPBDDriver::ReorderMaps(TBB_ThreadPool* threadPool, const Internal::PhysicsWorldContext& context)
    {
        auto func = [&]() {
            tbb::parallel_invoke(
                [&] { // directPosOffsets
                    directionPosOffsetsOrder.resize(directionPosOffsets.numOffsets);
                    std::iota(directionPosOffsetsOrder.begin(), directionPosOffsetsOrder.end(), 0);
                    std::ranges::sort(directionPosOffsetsOrder, [&](std::uint32_t a, std::uint32_t b) {
                        const std::uint32_t biSrcA = directionPosOffsets.srcBoneIdx[a];
                        const std::uint32_t biSrcB = directionPosOffsets.srcBoneIdx[b];
                        const std::uint32_t biDstA = directionPosOffsets.dstBoneIdx[a];
                        const std::uint32_t biDstB = directionPosOffsets.dstBoneIdx[b];
                        bool invalidBoneA = (biSrcA == UINT32_MAX) || (biDstA == UINT32_MAX);
                        bool invalidBoneB = (biSrcB == UINT32_MAX) || (biDstB == UINT32_MAX);
                        if (invalidBoneA || invalidBoneB)
                            return invalidBoneA < invalidBoneB;
                        if (directionPosOffsets.isWorldOffset[a] != directionPosOffsets.isWorldOffset[b])
                            return directionPosOffsets.isWorldOffset[a] < directionPosOffsets.isWorldOffset[b];
                        const std::uint32_t objIdxSrcA = context.physicsBones->objIdx[biSrcA];
                        const std::uint32_t objIdxSrcB = context.physicsBones->objIdx[biSrcB];
                        if (objIdxSrcA != objIdxSrcB)
                            return objIdxSrcA < objIdxSrcB;
                        const std::uint32_t objIdxDstA = context.physicsBones->objIdx[biDstA];
                        const std::uint32_t objIdxDstB = context.physicsBones->objIdx[biDstB];
                        if (objIdxDstA != objIdxDstB)
                            return objIdxDstA < objIdxDstB;
                        const std::uint32_t rootIdxSrcA = context.physicsBones->rootIdx[biSrcA];
                        const std::uint32_t rootIdxSrcB = context.physicsBones->rootIdx[biSrcB];
                        if (rootIdxSrcA != rootIdxSrcB)
                            return rootIdxSrcA < rootIdxSrcB;
                        const std::uint32_t rootIdxDstA = context.physicsBones->rootIdx[biDstA];
                        const std::uint32_t rootIdxDstB = context.physicsBones->rootIdx[biDstB];
                        if (rootIdxDstA != rootIdxDstB)
                            return rootIdxDstA < rootIdxDstB;
                        if (biSrcA != biSrcB)
                            return biSrcA < biSrcB;
                        return biDstA < biDstB;
                    });

                    std::uint32_t validCount = 0;
                    for (std::uint32_t coi = 0; coi < directionPosOffsets.numOffsets; ++coi)
                    {
                        const auto& oi = directionPosOffsetsOrder[coi];
                        if (directionPosOffsets.srcBoneIdx[oi] == UINT32_MAX ||
                            directionPosOffsets.dstBoneIdx[oi] == UINT32_MAX)
                            break;
                        validCount++;
                    }

                    {
                        const Driver::DirectionPosOffsets tmpDirectPosOffsets = directionPosOffsets;
                        tbb::parallel_for(
                            tbb::blocked_range<std::uint32_t>(0, validCount, 128),
                            [&](const tbb::blocked_range<std::uint32_t>& r) {
                                for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                                {
                                    const std::uint32_t srcIdx = directionPosOffsetsOrder[i];

                                    directionPosOffsets.driverName[i] = tmpDirectPosOffsets.driverName[srcIdx];
                                    directionPosOffsets.srcBoneIdx[i] = tmpDirectPosOffsets.srcBoneIdx[srcIdx];
                                    directionPosOffsets.dstBoneIdx[i] = tmpDirectPosOffsets.dstBoneIdx[srcIdx];
                                    directionPosOffsets.isWorldOffset[i] = tmpDirectPosOffsets.isWorldOffset[srcIdx];
                                    directionPosOffsets.baseDirection[i] = tmpDirectPosOffsets.baseDirection[srcIdx];
                                    directionPosOffsets.targetDirection[i] = tmpDirectPosOffsets.targetDirection[srcIdx];
                                    directionPosOffsets.minOffset[i] = tmpDirectPosOffsets.minOffset[srcIdx];
                                    directionPosOffsets.maxOffset[i] = tmpDirectPosOffsets.maxOffset[srcIdx];
                                }
                            },
                            tbb::static_partitioner());
                    }
                    directionPosOffsets.driverName.resize(validCount);
                    directionPosOffsets.srcBoneIdx.resize(validCount);
                    directionPosOffsets.dstBoneIdx.resize(validCount);
                    directionPosOffsets.isWorldOffset.resize(validCount);
                    directionPosOffsets.baseDirection.resize(validCount);
                    directionPosOffsets.targetDirection.resize(validCount);
                    directionPosOffsets.minOffset.resize(validCount);
                    directionPosOffsets.maxOffset.resize(validCount);

                    directionPosOffsets.numOffsets = validCount;
                    directionPosOffsetsOrder.resize(validCount);
                    std::iota(directionPosOffsetsOrder.begin(), directionPosOffsetsOrder.end(), 0);
                },
                [&] { // directRotOffsets
                    directionRotOffsetsOrder.resize(directionRotOffsets.numOffsets);
                    std::iota(directionRotOffsetsOrder.begin(), directionRotOffsetsOrder.end(), 0);
                    std::ranges::sort(directionRotOffsetsOrder, [&](std::uint32_t a, std::uint32_t b) {
                        const std::uint32_t biSrcA = directionRotOffsets.srcBoneIdx[a];
                        const std::uint32_t biSrcB = directionRotOffsets.srcBoneIdx[b];
                        const std::uint32_t biDstA = directionRotOffsets.dstBoneIdx[a];
                        const std::uint32_t biDstB = directionRotOffsets.dstBoneIdx[b];
                        bool invalidBoneA = (biSrcA == UINT32_MAX) || (biDstA == UINT32_MAX);
                        bool invalidBoneB = (biSrcB == UINT32_MAX) || (biDstB == UINT32_MAX);
                        if (invalidBoneA || invalidBoneB)
                            return invalidBoneA < invalidBoneB;
                        if (directionRotOffsets.isWorldOffset[a] != directionRotOffsets.isWorldOffset[b])
                            return directionRotOffsets.isWorldOffset[a] < directionRotOffsets.isWorldOffset[b];
                        const std::uint32_t objIdxSrcA = context.physicsBones->objIdx[biSrcA];
                        const std::uint32_t objIdxSrcB = context.physicsBones->objIdx[biSrcB];
                        if (objIdxSrcA != objIdxSrcB)
                            return objIdxSrcA < objIdxSrcB;
                        const std::uint32_t objIdxDstA = context.physicsBones->objIdx[biDstA];
                        const std::uint32_t objIdxDstB = context.physicsBones->objIdx[biDstB];
                        if (objIdxDstA != objIdxDstB)
                            return objIdxDstA < objIdxDstB;
                        const std::uint32_t rootIdxSrcA = context.physicsBones->rootIdx[biSrcA];
                        const std::uint32_t rootIdxSrcB = context.physicsBones->rootIdx[biSrcB];
                        if (rootIdxSrcA != rootIdxSrcB)
                            return rootIdxSrcA < rootIdxSrcB;
                        const std::uint32_t rootIdxDstA = context.physicsBones->rootIdx[biDstA];
                        const std::uint32_t rootIdxDstB = context.physicsBones->rootIdx[biDstB];
                        if (rootIdxDstA != rootIdxDstB)
                            return rootIdxDstA < rootIdxDstB;
                        if (biSrcA != biSrcB)
                            return biSrcA < biSrcB;
                        return biDstA < biDstB;
                    });

                    std::uint32_t validCount = 0;
                    for (std::uint32_t coi = 0; coi < directionRotOffsets.numOffsets; ++coi)
                    {
                        const auto& oi = directionRotOffsetsOrder[coi];
                        if (directionRotOffsets.srcBoneIdx[oi] == UINT32_MAX ||
                            directionRotOffsets.dstBoneIdx[oi] == UINT32_MAX)
                            break;
                        validCount++;
                    }

                    {
                        const Driver::DirectionRotOffsets tmpDirectRotOffsets = directionRotOffsets;
                        tbb::parallel_for(
                            tbb::blocked_range<std::uint32_t>(0, validCount, 128),
                            [&](const tbb::blocked_range<std::uint32_t>& r) {
                                for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                                {
                                    const std::uint32_t srcIdx = directionRotOffsetsOrder[i];

                                    directionRotOffsets.driverName[i] = tmpDirectRotOffsets.driverName[srcIdx];
                                    directionRotOffsets.srcBoneIdx[i] = tmpDirectRotOffsets.srcBoneIdx[srcIdx];
                                    directionRotOffsets.dstBoneIdx[i] = tmpDirectRotOffsets.dstBoneIdx[srcIdx];
                                    directionRotOffsets.isWorldOffset[i] = tmpDirectRotOffsets.isWorldOffset[srcIdx];
                                    directionRotOffsets.direct[i] = tmpDirectRotOffsets.direct[srcIdx];
                                    directionRotOffsets.minOffset[i] = tmpDirectRotOffsets.minOffset[srcIdx];
                                    directionRotOffsets.maxOffset[i] = tmpDirectRotOffsets.maxOffset[srcIdx];
                                }
                            },
                            tbb::static_partitioner());
                    }
                    directionRotOffsets.driverName.resize(validCount);
                    directionRotOffsets.srcBoneIdx.resize(validCount);
                    directionRotOffsets.dstBoneIdx.resize(validCount);
                    directionRotOffsets.isWorldOffset.resize(validCount);
                    directionRotOffsets.direct.resize(validCount);
                    directionRotOffsets.minOffset.resize(validCount);
                    directionRotOffsets.maxOffset.resize(validCount);

                    directionRotOffsets.numOffsets = validCount;
                    directionRotOffsetsOrder.resize(validCount);
                    std::iota(directionRotOffsetsOrder.begin(), directionRotOffsetsOrder.end(), 0);
                }
            );
            orderDirty = false;
        };

        if (threadPool)
            threadPool->Execute(func);
        else
            func();
    }

    void XPBDDriver::LoggingTimeProfiler(const std::string& funcName, const double ms) const
    {
        logger::debug("{} time: {:.3f}ms ({} directionPosOffsets / {} directionRotOffsets)", funcName, ms,
                      directionPosOffsets.numOffsets, directionRotOffsets.numOffsets);
    }
}