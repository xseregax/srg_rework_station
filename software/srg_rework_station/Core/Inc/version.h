
#pragma once

extern const char* GIT_TAG;
extern const char* GIT_REV;

const char* git_version(void)
{
    return GIT_TAG;
}

const char* git_revision(void)
{
    return GIT_REV;
}
