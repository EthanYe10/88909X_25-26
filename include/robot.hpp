#pragma once

void run_intake();
void reverse_intake();

inline void flip_hook();
inline void flip_match_loader();

inline void raise_hook();
inline void lower_hook();

inline void deploy_match_loader();
inline void retract_match_loader();

// driver control functions
void run_drivetrain();
void driver_intake();
void switch_intake_mode();
void get_pneumatic_inputs();
void run_driver();