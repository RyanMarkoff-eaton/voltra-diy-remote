#pragma once
#ifdef VOLTRA_DISPLAY
enum class UiSelection : uint8_t { None, Weight, Eccentric, Chains, InverseChains };
enum class UiDropEdit : uint8_t { None, Amount, Hold };
void uiBegin();
bool uiReady();
int uiTakeEncoderDelta();
UiSelection uiSelection();
UiSelection uiTakeModifierToggle();
bool uiTakeDropToggle();
bool uiTakeSleepToggle();
UiDropEdit uiDropEdit();
enum class UiButtonAction : uint8_t { None, ApplyWeight, GuidedLoad, Stop };
UiButtonAction uiTakeButtonAction();
void uiTick(int weight, int eccentric, int chains, int inverseChains,
            int confirmedWeight, int confirmedEccentric, int confirmedChains,
            int confirmedInverseChains, bool eccentricEnabled, bool chainsEnabled,
            bool inverseChainsEnabled, bool activationTriggered, bool dropEnabled,
            int dropAmount, int dropHoldSeconds, bool dropArmed, bool autoSleepEnabled, bool connected);
void uiPowerDown();
#else
enum class UiSelection : uint8_t { None, Weight, Eccentric, Chains, InverseChains };
enum class UiDropEdit : uint8_t { None, Amount, Hold };
inline void uiBegin() {}
inline bool uiReady() { return false; }
inline int uiTakeEncoderDelta() { return 0; }
inline UiSelection uiSelection() { return UiSelection::None; }
inline UiSelection uiTakeModifierToggle() { return UiSelection::None; }
inline bool uiTakeDropToggle() { return false; }
inline bool uiTakeSleepToggle() { return false; }
inline UiDropEdit uiDropEdit() { return UiDropEdit::None; }
enum class UiButtonAction : uint8_t { None, ApplyWeight, GuidedLoad, Stop };
inline UiButtonAction uiTakeButtonAction() { return UiButtonAction::None; }
inline void uiTick(int, int, int, int, int, int, int, int, bool, bool, bool, bool, bool, int, int, bool, bool, bool) {}
inline void uiPowerDown() {}
#endif
