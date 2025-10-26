# Security Policy

Thank you for helping keep K0I05/ESP32-S3_ESP-IDF_COMPONENTS secure. This document explains how to report security vulnerabilities, what we will do after receiving a report, and the kind of issues that are in- or out-of-scope for this repository.

## Reporting a vulnerability (preferred: private disclosure)

Preferred method (recommended)
1. Use GitHub's private Security Advisories:
   - In the repository, go to Security → Advisories → New draft security advisory.
   - Provide as much information as possible (see the "What to include" section below).
   - We will respond and coordinate fixes privately via the advisory.

Alternative methods
- If you cannot use GitHub Security Advisories, contact the repository maintainer via a direct GitHub message to @K0I05 and indicate that you have a vulnerability report that should not be made public.
- DO NOT open a public issue containing sensitive vulnerability details. If you must open an issue to notify us, use a short title such as "SECURITY: <brief summary>" and do not include exploit details; follow up with a private channel (see above).

If you need to send sensitive data via email or direct message, include a PGP/GPG public key in your first message and we will provide a key to encrypt sensitive details. If you do not provide a key, do not include PoC exploit code or sensitive data in public channels.

## What to include with your report

Please provide:
- Brief summary and classification of the issue (e.g., buffer overflow, authentication bypass).
- Repository path(s) and version(s)/commit SHA(s) affected.
- Step-by-step reproduction steps or Proof-of-Concept (PoC) if available. If you cannot share PoC publicly, include it in the private advisory or encrypted message.
- Impact: what an attacker can do (data leak, remote code execution, device compromise, etc.).
- CVSS score if you have one (optional).
- Suggested remediation or an idea for a fix if you have one.
- Contact information and your disclosure preference (e.g., embargo duration or coordinate public release).
- Your PGP/GPG public key (if you want the response encrypted).

Example report header:
- Title: Command injection in component XYZ causing remote code execution
- Affected version(s): main branch at commit 0123abcd, tag v1.2.0
- Steps to reproduce: ...
- Impact: ...
- Contact: @username on GitHub / email@example.com
- PGP public key (optional): -----BEGIN PGP PUBLIC KEY BLOCK-----

## Scope

In-scope
- Code, examples, build scripts, and CI workflows contained in this repository.
- Releases and tags created from this repository.

Out-of-scope
- Third-party dependencies not maintained by this project (report them to the component maintainers and/or their upstream projects).
- Vulnerabilities in downstream applications unless they originate in code from this repository.

If you are unsure whether something is in-scope, contact us privately.

## Our commitments and timelines

Acknowledgement
- We will try to acknowledge receipt of your report within 48 hours of receiving it (weekends/holidays may affect timings).

Triage
- We will triage and begin investigating within 7 days of acknowledgment.

Fixing & disclosure
- Fix timing depends on severity and complexity:
  - Critical (e.g., unauthenticated remote code execution) — expedited handling; we will aim to provide a patch or mitigation as soon as practicable and coordinate disclosure.
  - High/Medium/Low — we will prioritize according to severity and impact; expect progress updates during triage.
- We aim to publish a coordinated public advisory and release a fixed version within 90 days of initial report for non-critical issues. Critical vulnerabilities may have shorter embargo periods based on risk and coordination.
- In all cases we will coordinate with the reporter about public disclosure timing. If you request a longer embargo, we will do our best to accommodate it and will communicate any constraints.

CVE assignment
- We will request a CVE identifier for vulnerabilities that meet criteria for CVE assignment and will include it in the advisory and release notes.

Updates and patches
- Security fixes will be issued as a patch release and a public GitHub advisory once coordinated disclosure is complete. The repository CHANGELOG and release notes will include a brief advisory summary and CVE number (if assigned).

## Safe harbor for security researchers

We appreciate responsible disclosure. We will not pursue legal action against those who follow this policy and act in good faith to report security vulnerabilities and do not violate applicable laws. This does not apply to malicious or negligent behavior (for example, intentionally exploiting vulnerabilities against third parties, exfiltrating data, or performing denial-of-service attacks).

## Disclosure and communication

When a fix is ready and disclosure is coordinated:
- We will publish a GitHub Security Advisory and a release (tag) containing the fix.
- The advisory will describe the issue, affected versions, severity, mitigation steps, and references (including CVE number when available).
- Credit will be given to the reporter if they wish to be credited (by name or handle).

## Non-security issues

For feature requests, bugs that are not security-related, or general support, please use the Issues tab or Discussions (if enabled). Avoid adding security-sensitive information there.

## Contact & escalation

Primary contact
- Preferred: Create a private GitHub Security Advisory in this repository (Security → Advisories → New draft security advisory).
- Alternate: Direct message the repository owner/maintainer @K0I05 on GitHub for private coordination.

If you do not receive an acknowledgement within 48 hours, repeat the contact via the other listed method. If you feel escalation is necessary after that point, state so in your report and we will escalate internally.

## Acknowledgements

We appreciate the time and effort of security researchers who responsibly disclose vulnerabilities. We will credit contributors who request recognition and whose reports lead to improvements.

Thank you for helping keep this project secure.

Copyright (c) 2025 Eric Gionet (<gionet.c.eric@gmail.com>)
