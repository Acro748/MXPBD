#include "Store.h"

namespace Mus {
	std::atomic<bool> IsRaceSexMenu = false;
	std::atomic<bool> IsMainMenu = false;
	std::atomic<bool> IsGamePaused = false;
	std::atomic<bool> IsSaveLoading = false;
    std::atomic<bool> IsInLoading = false;
}
