# Contributing

Want to contribute? Great! You can do so through the standard GitHub pull
request model. For large contributions we do encourage you to file a ticket in
the GitHub issues tracking system prior to any code development to coordinate
with the UUV Simulator development team early in the process. Coordinating up
front helps to avoid frustration later on. Please properly document your code
and follow the [Google C++ Style Guide][StyleGuide]. You can check style guide
compliance by running ```./tools/code_check.sh```

Your contribution should be licensed under the Apache-2.0 license, the license
used by this project.

## Sign your work

This project tracks patch provenance and licensing using a modified Developer
Certificate of Origin (from [OSDL][DCO]) and Signed-off-by tags initially
developed by the Linux kernel project.

```
UUV Simulator Developer's Certificate of Origin.  Version 1.0

By making a contribution to this project, I certify that:

(a) The contribution was created in whole or in part by me and I
    have the right to submit it under the license of "Apache License,
    Version 2.0" ("Apache-2.0"); or

(b) The contribution is based upon previous work that is covered by
    an appropriate open source license and I have the right under
    that license to submit that work with modifications, whether
    created in whole or in part by me, under the Apache-2.0 license;
    or

(c) The contribution was provided directly to me by some other
    person who certified (a), (b) and I have not modified it.

(d) I understand and agree that this project and the contribution
    are public and that a record of the contribution (including all
    metadata and personal information I submit with it, including my
    sign-off) is maintained indefinitely and may be redistributed
    consistent with this project and the requirements of the Apache-2.0
    license or any open source license(s) involved, where they are
    relevant.

(e) I am granting the contribution to this project under the terms of
    Apache-2.0.

    http://www.apache.org/licenses/LICENSE-2.0
```

With the sign-off in a commit message you certify that you authored the patch
or otherwise have the right to submit it under an open source license. The
procedure is simple: To certify above UUV Simulator Developer's Certificate of
Origin 1.0 for your contribution just append a line

    Signed-off-by: Random J Developer <random@developer.example.org>

to every commit message using your real name (sorry, no pseudonyms or
anonymous contributions).

If you have set your `user.name` and `user.email` git configs you can
automatically sign the commit by running the git-commit command with the `-s`
option.  There may be multiple sign-offs if more than one developer was
involved in authoring the contribution.

For a more detailed description of this procedure, please see
[SubmittingPatches][] which was extracted from the Linux kernel project, and
which is stored in an external repository.

### Individual vs. Corporate Contributors

Often employers or academic institution claim ownership over code that is
written in certain circumstances, so please do due diligence to ensure that
you have the right to submit the code.

If you are a developer who is authorized to contribute to UUV Simulator on
behalf of your employer, then please use your corporate email address in the
Signed-off-by tag. Otherwise please use a personal email address.

## Maintain Contributors and Authors lists

Each contributor is responsible for identifying themselves in the project
[CONTRIBUTORS](CONTRIBUTORS) and [AUTHORS](AUTHORS) lists. Please add the
respective information corresponding to the Singed-off-by tag to those files
as part of your first pull request.

If you are a developer who is authorized to contribute to UUV Simulator on
behalf of your employer, then add your company / organization to
[AUTHORS](AUTHORS) and your name and corporate email address to
[CONTRIBUTORS](CONTRIBUTORS) as in the Signed-off-by tag.


[DCO]: http://web.archive.org/web/20070306195036/http://osdlab.org/newsroom/press_releases/2004/2004_05_24_dco.html

[SubmittingPatches]: https://github.com/wking/signed-off-by/blob/7d71be37194df05c349157a2161c7534feaf86a4/Documentation/SubmittingPatches

[StyleGuide]: [https://google.github.io/styleguide/cppguide.html]
