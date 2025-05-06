# Coding Conventions used in the Self Balancing Robot Framework

The coding conventions used are guidelines, not tramlines. That means they are generally adhered to, but can be broken it makes the code more understandable.

## Naming conventions

1. Generally the *camelCase* naming convention is used, with class names starting with a capital letter, and procedure and data names starting with a
   lower case letter.
2. When an abbreviation is used within a name, it is followed by an underscore. So we have *AHRS_Base*, not *AHRSBase* or *AhrsBase*.
3. When external libraries are used, the naming convention of that library is adopted.

## Prefixes and suffixes

When there are several objects of the same type, then they are grouped together using either a common prefix or common suffix. For example *encoderLeft* and *encoderRight*.

The choice of prefix or suffix depends on how things are typically dealt with. So we have *encoderLeft* and *encoderRight*,
since generally all things related to encoders are done together. However we have *pitchPID*, *speedPID*, and *yawRatePID*
(rather than *PID_pitch*, *PID_speed*, and *PID_yawRate*) since generally we deal with all things related to pitch together,
(rather than dealing with all things related to PIDs together). This choice is not always clear cut, however.

## Abbreviations

A policy of "no unnecessary abbreviations" is adopted. This means names are spelt out in full, so we have *filter* not *fltr*, and *screen* not *scrn*.

When there are generally established abbreviations, they are used, for example: *PID* and *IMU*.

When external libraries are used, any abbreviations used in that library are retained.

## Magic booleans

Using magic booleans, especially functions that take a parameter that takes the value `true` or `false` can make code confusing, and often means you
have to refer to the documentation to figure out what the code does..

For example consider `preferences.begin("myPreferences", false);`: without looking it up, you no idea of what `false` signifies. This is easy to rectify
just create an `enum` that defines the parameter options in a meaningful way, eg `enum { READ_WRITE=false, READ_ONLY=true };` then this aforementioned
code becomes `preferences.begin("myPreferences", READ_WRITE);` and its meaning is clear.
