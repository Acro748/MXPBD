
namespace {
    void InitializeLogging() 
    {
        std::string LogDirectory = Mus::GetRuntimeSKSEDirectory();
        
        std::optional<std::filesystem::path> path(LogDirectory);
        *path /= "MXPBD";
        *path += L".log";

        std::shared_ptr<spdlog::logger> log;
        if (IsDebuggerPresent()) 
        {
            log = std::make_shared<spdlog::logger>(
                "Global", std::make_shared<spdlog::sinks::msvc_sink_mt>());
        } 
        else 
        {
            log = std::make_shared<spdlog::logger>(
                "Global", std::make_shared<spdlog::sinks::basic_file_sink_mt>(path->string(), true));
        }

        log->set_level(Mus::Config::GetSingleton().GetLogLevel());
        log->flush_on(Mus::Config::GetSingleton().GetFlushLevel());

        spdlog::set_default_logger(std::move(log));
        spdlog::set_pattern("[%H:%M:%S.%e][%L][%t][%s:%#] %v");
    }

    void AllSave(SKSE::SerializationInterface* serde)
    {
    }

    void AllLoad(SKSE::SerializationInterface* serde)
    {
        logger::info("save data loading...");

        std::uint32_t type;
        std::uint32_t size;
        std::uint32_t version;
        while (serde->GetNextRecordInfo(type, version, size)) {
        }
        logger::info("save data loaded");
    }

    void InitializeSerialization() 
    {
        logger::trace("Initializing cosave serialization...");
        auto* serde = SKSE::GetSerializationInterface();
        serde->SetUniqueID(_byteswap_ulong('XPBD'));
        serde->SetSaveCallback(AllSave);
        serde->SetLoadCallback(AllLoad);
    }

    void InitializePapyrus() 
    {
        /*logger::trace("Initializing Papyrus binding...");
        if (SKSE::GetPapyrusInterface()->Register(Mus::Papyrus::RegisterPapyrusFunctions))
        {
            logger::debug("Papyrus functions bound.");
        } 
        else 
        {
            SKSE::stl::report_and_fail("Failure to register Papyrus bindings.");
        }*/
    }

    void InitializeHooking() 
    {
        logger::trace("Building hook...");

        Mus::hook();
        Mus::Commandhook();
    }

    void kPostLoadFunction()
    {

    }
    void kDataloadedFunction()
    {
        Mus::Config::GetSingleton().LoadConfig();
        Mus::Config::GetSingleton().LoadSkeletonFile();
        Mus::ConditionManager::GetSingleton().SortConditions();
        Mus::Config::GetSingleton().LoadSMPDefaultConfig();
        MXPBD::IsHDTSMPEnabled = GetModuleHandleW(L"hdtSMP64");
        if (!MXPBD::IsHDTSMPEnabled)
            Mus::fixFaceGenBoneLimit();
        MXPBD::XPBDWorldSystem::GetSingleton().Init();
        MXPBD::XPBDWorldUpdater::GetSingleton().Init();
    }

    void kNewGameFunction()
    {
        logger::info("detected new game...");
    }

    void InitializeMessaging() 
    {
        if (!SKSE::GetMessagingInterface()->RegisterListener([](SKSE::MessagingInterface::Message* message) 
        {
            switch (message->type) 
            {
                // Skyrim lifecycle events.
                case SKSE::MessagingInterface::kPostLoad: // Called after all plugins have finished running SKSEPlugin_Load.
                    // It is now safe to do multithreaded operations, or operations against other plugins.
                    kPostLoadFunction();
                    break;
                case SKSE::MessagingInterface::kPostPostLoad: // Called after all kPostLoad message handlers have run.
                    break;
                case SKSE::MessagingInterface::kInputLoaded: // Called when all game data has been found.
                    break;
                case SKSE::MessagingInterface::kDataLoaded: // All ESM/ESL/ESP plugins have loaded, main menu is now active.
                    // It is now safe to access form data.
                    kDataloadedFunction();
                    break;

                // Skyrim game events.
                case SKSE::MessagingInterface::kNewGame: // Player starts a new game from main menu.
                    kNewGameFunction();
                    break;
                case SKSE::MessagingInterface::kPreLoadGame: // Player selected a game to load, but it hasn't loaded yet.
                    Mus::IsSaveLoading.store(true);
                    // Data will be the name of the loaded save.
                    break;
                case SKSE::MessagingInterface::kPostLoadGame: // Player's selected save game has finished loading.
                    Mus::IsSaveLoading.store(false);
                    // Data will be a boolean indicating whether the load was successful.
                    break;
                case SKSE::MessagingInterface::kSaveGame: // The player has saved a game.
                    // Data will be the save name.
                    break;
                case SKSE::MessagingInterface::kDeleteGame: // The player deleted a saved game from within the load menu.
                    break;
            }
        })) {
            SKSE::stl::report_and_fail("Unable to register message listener.");
        }
    }
}

SKSEPluginLoad(const SKSE::LoadInterface* skse) 
{
    Mus::Config::GetSingleton().LoadLogging();
    InitializeLogging();

    auto* plugin = SKSE::PluginDeclaration::GetSingleton();
    auto version = plugin->GetVersion();
    logger::info("{} {} is loading...", plugin->GetName(), version);

    auto runtime = REL::Module::get().version();
    logger::info("Working on skyrim version : {}.{}.{}.{}", runtime.major(), runtime.minor(), runtime.patch(), runtime.build());

    Init(skse, false);
    InitializeMessaging();
    //InitializeSerialization();
    //InitializePapyrus();
    InitializeHooking();

    logger::info("{} has finished loading.", plugin->GetName());
    return true;
}
