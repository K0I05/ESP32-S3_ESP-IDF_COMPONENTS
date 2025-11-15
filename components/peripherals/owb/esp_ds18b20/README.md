# Maxim-Integrated DS18B20 Sensor

[![K0I05](https://img.shields.io/badge/K0I05-a9a9a9?logo=data:image/svg%2bxml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIxODgiIGhlaWdodD0iMTg3Ij48cGF0aCBmaWxsPSIjNDU0QjU0IiBkPSJNMTU1LjU1NSAyMS45M2MxOS4yNzMgMTUuOTggMjkuNDcyIDM5LjM0NSAzMi4xNjggNjMuNzg5IDEuOTM3IDIyLjkxOC00LjU1MyA0Ni42Ni0xOC44NDggNjQuNzgxQTUwOS40NzggNTA5LjQ3OCAwIDAgMSAxNjUgMTU1bC0xLjQ4NCAxLjg4M2MtMTMuMTk2IDE2LjUzMS0zNS41NTUgMjcuMjE1LTU2LjMzOSAyOS45MDItMjguMzEyIDIuOC01Mi4yNTUtNC43MzctNzQuNzMyLTIxLjcxNUMxMy4xNzIgMTQ5LjA5IDIuOTczIDEyNS43MjUuMjc3IDEwMS4yODEtMS42NiA3OC4zNjMgNC44MyA1NC42MjEgMTkuMTI1IDM2LjVBNTA5LjQ3OCA1MDkuNDc4IDAgMCAxIDIzIDMybDEuNDg0LTEuODgzQzM3LjY4IDEzLjU4NiA2MC4wNCAyLjkwMiA4MC44MjMuMjE1YzI4LjMxMi0yLjggNTIuMjU1IDQuNzM3IDc0LjczMiAyMS43MTVaIi8+PHBhdGggZmlsbD0iI0ZERkRGRCIgZD0iTTExOS44NjcgNDUuMjdDMTI4LjkzMiA1Mi4yNiAxMzMuODIgNjMgMTM2IDc0Yy42MyA0Ljk3Mi44NDIgOS45NTMuOTUzIDE0Ljk2LjA0NCAxLjkxMS4xMjIgMy44MjIuMjAzIDUuNzMxLjM0IDEyLjIxLjM0IDEyLjIxLTMuMTU2IDE3LjMwOWE5NS42MDQgOTUuNjA0IDAgMCAxLTQuMTg4IDMuNjI1Yy00LjUgMy43MTctNi45NzQgNy42ODgtOS43MTcgMTIuODAzQzEwNi45NCAxNTIuNzkyIDEwNi45NCAxNTIuNzkyIDk3IDE1N2MtMy40MjMuNTkyLTUuODAxLjY4NS04Ljg3OS0xLjA3NC05LjgyNi03Ljg4LTE2LjAzNi0xOS41OS0yMS44NTgtMzAuNTEyLTIuNTM0LTQuNTc1LTUuMDA2LTcuMjEtOS40NjYtMTAuMDItMy43MTQtMi44ODItNS40NS02Ljk4Ni02Ljc5Ny0xMS4zOTQtLjU1LTQuODg5LS41NjEtOS4zMTYgMS0xNCAuMDkzLTEuNzYzLjE4Mi0zLjUyNy4yMzktNS4yOTIuNDkxLTEzLjg4NCAzLjg2Ni0yNy4wNTcgMTQuMTU2LTM3LjAyOCAxNy4yMTgtMTQuMzM2IDM1Ljg1OC0xNS4wNjYgNTQuNDcyLTIuNDFaIi8+PHBhdGggZmlsbD0iI0M2RDVFMCIgZD0iTTEwOSAzOWMxMS43MDMgNS4yNTUgMTkuMjA2IDEzLjE4NiAyNC4yOTMgMjUuMDA0IDIuODU3IDguMjQgMy40NyAxNi4zMTYgMy42NiAyNC45NTYuMDQ0IDEuOTExLjEyMiAzLjgyMi4yMDMgNS43MzEuMzQgMTIuMjEuMzQgMTIuMjEtMy4xNTYgMTcuMzA5YTk1LjYwNCA5NS42MDQgMCAwIDEtNC4xODggMy42MjVjLTQuNSAzLjcxNy02Ljk3NCA3LjY4OC05LjcxNyAxMi44MDNDMTA2LjgwNCAxNTMuMDQxIDEwNi44MDQgMTUzLjA0MSA5NyAxNTdjLTIuMzMyLjA3OC00LjY2OC4wOS03IDBsMi4xMjUtMS44NzVjNS40My01LjQ0NSA4Ljc0NC0xMi41NzcgMTEuNzU0LTE5LjU1OWEzNDkuNzc1IDM0OS43NzUgMCAwIDEgNC40OTYtOS44NzlsMS42NDgtMy41NWMyLjI0LTMuNTU1IDQuNDEtNC45OTYgNy45NzctNy4xMzcgMi4zMjMtMi42MSAyLjMyMy0yLjYxIDQtNWwtMyAxYy0yLjY4LjE0OC01LjMxOS4yMy04IC4yNWwtMi4xOTUuMDYzYy01LjI4Ny4wMzktNS4yODcuMDM5LTcuNzc4LTEuNjUzLTEuNjY2LTIuNjkyLTEuNDUzLTQuNTYtMS4wMjctNy42NiAyLjM5NS00LjM2MiA0LjkyNC04LjA0IDkuODI4LTkuNTcgMi4zNjQtLjQ2OCA0LjUxNC0uNTI4IDYuOTIyLS40OTNsMi40MjIuMDI4TDEyMSA5MmwtMS0yYTkyLjc1OCA5Mi43NTggMCAwIDEtLjM2LTQuNTg2QzExOC42IDY5LjYzMiAxMTYuNTE3IDU2LjA5NCAxMDQgNDVjLTUuOTA0LTQuNjY0LTExLjYtNi4wODgtMTktNyA3LjU5NC00LjI2NCAxNi4yMjMtMS44MSAyNCAxWiIvPjxwYXRoIGZpbGw9IiM0OTUwNTgiIGQ9Ik03NyA5MmM0LjYxMyAxLjY3MSA3LjI2IDMuOTQ1IDEwLjA2MyA3LjkzOCAxLjA3OCAzLjUyMy45NzYgNS41NDYtLjA2MyA5LjA2Mi0yLjk4NCAyLjk4NC02LjI1NiAyLjM2OC0xMC4yNSAyLjM3NWwtMi4yNzcuMDc0Yy01LjI5OC4wMjgtOC4yNTQtLjk4My0xMi40NzMtNC40NDktMi44MjYtMy41OTctMi40MTYtNy42MzQtMi0xMiA0LjUwMi00LjcyOCAxMC45OS0zLjc2IDE3LTNaIi8+PHBhdGggZmlsbD0iIzQ4NEY1NyIgZD0ibTExOCA5MS43NSAzLjEyNS0uMDc4YzMuMjU0LjM3MSA0LjU5NyAxLjAwMiA2Ljg3NSAzLjMyOC42MzkgNC4yMzEuMjkgNi40NDItMS42ODggMTAuMjUtMy40MjggNC4wNzgtNS44MjcgNS41OTgtMTEuMTk1IDYuMTQ4LTEuNDE0LjAwOC0yLjgyOCAwLTQuMjQyLS4wMjNsLTIuMTY4LjAzNWMtMi45OTgtLjAxNy01LjE1Ny0uMDMzLTcuNjcyLTEuNzU4LTEuNjgxLTIuNjg0LTEuNDYtNC41NTItMS4wMzUtNy42NTIgMi4zNzUtNC4zMjUgNC44OTQtOC4wMDkgOS43NS05LjU1OSAyLjc3Ny0uNTQ0IDUuNDItLjY0OSA4LjI1LS42OTFaIi8+PHBhdGggZmlsbD0iIzUyNTg2MCIgZD0iTTg2IDEzNGgxNmwxIDRjLTIgMi0yIDItNS4xODggMi4yNjZMOTQgMTQwLjI1bC0zLjgxMy4wMTZDODcgMTQwIDg3IDE0MCA4NSAxMzhsMS00WiIvPjwvc3ZnPg==)](https://github.com/K0I05)
[![License: MIT](https://cdn.prod.website-files.com/5e0f1144930a8bc8aace526c/65dd9eb5aaca434fac4f1c34_License-MIT-blue.svg)](/LICENSE)
[![Language](https://img.shields.io/badge/Language-C-navy.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Framework](https://img.shields.io/badge/Framework-ESP_IDF-red.svg)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)
[![Edited with VS Code](https://img.shields.io/badge/Edited_with-VS_Code-blue?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMTAwIiBoZWlnaHQ9IjEwMCIgdmlld0JveD0iMCAwIDEwMCAxMDAiIGZpbGw9Im5vbmUiIHhtbG5zPSJodHRwOi8vd3d3LnczLm9yZy8yMDAwL3N2ZyI%2BCjxtYXNrIGlkPSJtYXNrMCIgbWFzay10eXBlPSJhbHBoYSIgbWFza1VuaXRzPSJ1c2VyU3BhY2VPblVzZSIgeD0iMCIgeT0iMCIgd2lkdGg9IjEwMCIgaGVpZ2h0PSIxMDAiPgo8cGF0aCBmaWxsLXJ1bGU9ImV2ZW5vZGQiIGNsaXAtcnVsZT0iZXZlbm9kZCIgZD0iTTcwLjkxMTkgOTkuMzE3MUM3Mi40ODY5IDk5LjkzMDcgNzQuMjgyOCA5OS44OTE0IDc1Ljg3MjUgOTkuMTI2NEw5Ni40NjA4IDg5LjIxOTdDOTguNjI0MiA4OC4xNzg3IDEwMCA4NS45ODkyIDEwMCA4My41ODcyVjE2LjQxMzNDMTAwIDE0LjAxMTMgOTguNjI0MyAxMS44MjE4IDk2LjQ2MDkgMTAuNzgwOEw3NS44NzI1IDAuODczNzU2QzczLjc4NjIgLTAuMTMwMTI5IDcxLjM0NDYgMC4xMTU3NiA2OS41MTM1IDEuNDQ2OTVDNjkuMjUyIDEuNjM3MTEgNjkuMDAyOCAxLjg0OTQzIDY4Ljc2OSAyLjA4MzQxTDI5LjM1NTEgMzguMDQxNUwxMi4xODcyIDI1LjAwOTZDMTAuNTg5IDIzLjc5NjUgOC4zNTM2MyAyMy44OTU5IDYuODY5MzMgMjUuMjQ2MUwxLjM2MzAzIDMwLjI1NDlDLTAuNDUyNTUyIDMxLjkwNjQgLTAuNDU0NjMzIDM0Ljc2MjcgMS4zNTg1MyAzNi40MTdMMTYuMjQ3MSA1MC4wMDAxTDEuMzU4NTMgNjMuNTgzMkMtMC40NTQ2MzMgNjUuMjM3NCAtMC40NTI1NTIgNjguMDkzOCAxLjM2MzAzIDY5Ljc0NTNMNi44NjkzMyA3NC43NTQxQzguMzUzNjMgNzYuMTA0MyAxMC41ODkgNzYuMjAzNyAxMi4xODcyIDc0Ljk5MDVMMjkuMzU1MSA2MS45NTg3TDY4Ljc2OSA5Ny45MTY3QzY5LjM5MjUgOTguNTQwNiA3MC4xMjQ2IDk5LjAxMDQgNzAuOTExOSA5OS4zMTcxWk03NS4wMTUyIDI3LjI5ODlMNDUuMTA5MSA1MC4wMDAxTDc1LjAxNTIgNzIuNzAxMlYyNy4yOTg5WiIgZmlsbD0id2hpdGUiLz4KPC9tYXNrPgo8ZyBtYXNrPSJ1cmwoI21hc2swKSI%2BCjxwYXRoIGQ9Ik05Ni40NjE0IDEwLjc5NjJMNzUuODU2OSAwLjg3NTU0MkM3My40NzE5IC0wLjI3Mjc3MyA3MC42MjE3IDAuMjExNjExIDY4Ljc1IDIuMDgzMzNMMS4yOTg1OCA2My41ODMyQy0wLjUxNTY5MyA2NS4yMzczIC0wLjUxMzYwNyA2OC4wOTM3IDEuMzAzMDggNjkuNzQ1Mkw2LjgxMjcyIDc0Ljc1NEM4LjI5NzkzIDc2LjEwNDIgMTAuNTM0NyA3Ni4yMDM2IDEyLjEzMzggNzQuOTkwNUw5My4zNjA5IDEzLjM2OTlDOTYuMDg2IDExLjMwMjYgMTAwIDEzLjI0NjIgMTAwIDE2LjY2NjdWMTYuNDI3NUMxMDAgMTQuMDI2NSA5OC42MjQ2IDExLjgzNzggOTYuNDYxNCAxMC43OTYyWiIgZmlsbD0iIzAwNjVBOSIvPgo8ZyBmaWx0ZXI9InVybCgjZmlsdGVyMF9kKSI%2BCjxwYXRoIGQ9Ik05Ni40NjE0IDg5LjIwMzhMNzUuODU2OSA5OS4xMjQ1QzczLjQ3MTkgMTAwLjI3MyA3MC42MjE3IDk5Ljc4ODQgNjguNzUgOTcuOTE2N0wxLjI5ODU4IDM2LjQxNjlDLTAuNTE1NjkzIDM0Ljc2MjcgLTAuNTEzNjA3IDMxLjkwNjMgMS4zMDMwOCAzMC4yNTQ4TDYuODEyNzIgMjUuMjQ2QzguMjk3OTMgMjMuODk1OCAxMC41MzQ3IDIzLjc5NjQgMTIuMTMzOCAyNS4wMDk1TDkzLjM2MDkgODYuNjMwMUM5Ni4wODYgODguNjk3NCAxMDAgODYuNzUzOCAxMDAgODMuMzMzNFY4My41NzI2QzEwMCA4NS45NzM1IDk4LjYyNDYgODguMTYyMiA5Ni40NjE0IDg5LjIwMzhaIiBmaWxsPSIjMDA3QUNDIi8%2BCjwvZz4KPGcgZmlsdGVyPSJ1cmwoI2ZpbHRlcjFfZCkiPgo8cGF0aCBkPSJNNzUuODU3OCA5OS4xMjYzQzczLjQ3MjEgMTAwLjI3NCA3MC42MjE5IDk5Ljc4ODUgNjguNzUgOTcuOTE2NkM3MS4wNTY0IDEwMC4yMjMgNzUgOTguNTg5NSA3NSA5NS4zMjc4VjQuNjcyMTNDNzUgMS40MTAzOSA3MS4wNTY0IC0wLjIyMzEwNiA2OC43NSAyLjA4MzI5QzcwLjYyMTkgMC4yMTE0MDIgNzMuNDcyMSAtMC4yNzM2NjYgNzUuODU3OCAwLjg3MzYzM0w5Ni40NTg3IDEwLjc4MDdDOTguNjIzNCAxMS44MjE3IDEwMCAxNC4wMTEyIDEwMCAxNi40MTMyVjgzLjU4NzFDMTAwIDg1Ljk4OTEgOTguNjIzNCA4OC4xNzg2IDk2LjQ1ODYgODkuMjE5Nkw3NS44NTc4IDk5LjEyNjNaIiBmaWxsPSIjMUY5Q0YwIi8%2BCjwvZz4KPGcgc3R5bGU9Im1peC1ibGVuZC1tb2RlOm92ZXJsYXkiIG9wYWNpdHk9IjAuMjUiPgo8cGF0aCBmaWxsLXJ1bGU9ImV2ZW5vZGQiIGNsaXAtcnVsZT0iZXZlbm9kZCIgZD0iTTcwLjg1MTEgOTkuMzE3MUM3Mi40MjYxIDk5LjkzMDYgNzQuMjIyMSA5OS44OTEzIDc1LjgxMTcgOTkuMTI2NEw5Ni40IDg5LjIxOTdDOTguNTYzNCA4OC4xNzg3IDk5LjkzOTIgODUuOTg5MiA5OS45MzkyIDgzLjU4NzFWMTYuNDEzM0M5OS45MzkyIDE0LjAxMTIgOTguNTYzNSAxMS44MjE3IDk2LjQwMDEgMTAuNzgwN0w3NS44MTE3IDAuODczNjk1QzczLjcyNTUgLTAuMTMwMTkgNzEuMjgzOCAwLjExNTY5OSA2OS40NTI3IDEuNDQ2ODhDNjkuMTkxMiAxLjYzNzA1IDY4Ljk0MiAxLjg0OTM3IDY4LjcwODIgMi4wODMzNUwyOS4yOTQzIDM4LjA0MTRMMTIuMTI2NCAyNS4wMDk2QzEwLjUyODMgMjMuNzk2NCA4LjI5Mjg1IDIzLjg5NTkgNi44MDg1NSAyNS4yNDZMMS4zMDIyNSAzMC4yNTQ4Qy0wLjUxMzMzNCAzMS45MDY0IC0wLjUxNTQxNSAzNC43NjI3IDEuMjk3NzUgMzYuNDE2OUwxNi4xODYzIDUwTDEuMjk3NzUgNjMuNTgzMkMtMC41MTU0MTUgNjUuMjM3NCAtMC41MTMzMzQgNjguMDkzNyAxLjMwMjI1IDY5Ljc0NTJMNi44MDg1NSA3NC43NTRDOC4yOTI4NSA3Ni4xMDQyIDEwLjUyODMgNzYuMjAzNiAxMi4xMjY0IDc0Ljk5MDVMMjkuMjk0MyA2MS45NTg2TDY4LjcwODIgOTcuOTE2N0M2OS4zMzE3IDk4LjU0MDUgNzAuMDYzOCA5OS4wMTA0IDcwLjg1MTEgOTkuMzE3MVpNNzQuOTU0NCAyNy4yOTg5TDQ1LjA0ODMgNTBMNzQuOTU0NCA3Mi43MDEyVjI3LjI5ODlaIiBmaWxsPSJ1cmwoI3BhaW50MF9saW5lYXIpIi8%2BCjwvZz4KPC9nPgo8ZGVmcz4KPGZpbHRlciBpZD0iZmlsdGVyMF9kIiB4PSItOC4zOTQxMSIgeT0iMTUuODI5MSIgd2lkdGg9IjExNi43MjciIGhlaWdodD0iOTIuMjQ1NiIgZmlsdGVyVW5pdHM9InVzZXJTcGFjZU9uVXNlIiBjb2xvci1pbnRlcnBvbGF0aW9uLWZpbHRlcnM9InNSR0IiPgo8ZmVGbG9vZCBmbG9vZC1vcGFjaXR5PSIwIiByZXN1bHQ9IkJhY2tncm91bmRJbWFnZUZpeCIvPgo8ZmVDb2xvck1hdHJpeCBpbj0iU291cmNlQWxwaGEiIHR5cGU9Im1hdHJpeCIgdmFsdWVzPSIwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAxMjcgMCIvPgo8ZmVPZmZzZXQvPgo8ZmVHYXVzc2lhbkJsdXIgc3RkRGV2aWF0aW9uPSI0LjE2NjY3Ii8%2BCjxmZUNvbG9yTWF0cml4IHR5cGU9Im1hdHJpeCIgdmFsdWVzPSIwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwLjI1IDAiLz4KPGZlQmxlbmQgbW9kZT0ib3ZlcmxheSIgaW4yPSJCYWNrZ3JvdW5kSW1hZ2VGaXgiIHJlc3VsdD0iZWZmZWN0MV9kcm9wU2hhZG93Ii8%2BCjxmZUJsZW5kIG1vZGU9Im5vcm1hbCIgaW49IlNvdXJjZUdyYXBoaWMiIGluMj0iZWZmZWN0MV9kcm9wU2hhZG93IiByZXN1bHQ9InNoYXBlIi8%2BCjwvZmlsdGVyPgo8ZmlsdGVyIGlkPSJmaWx0ZXIxX2QiIHg9IjYwLjQxNjciIHk9Ii04LjA3NTU4IiB3aWR0aD0iNDcuOTE2NyIgaGVpZ2h0PSIxMTYuMTUxIiBmaWx0ZXJVbml0cz0idXNlclNwYWNlT25Vc2UiIGNvbG9yLWludGVycG9sYXRpb24tZmlsdGVycz0ic1JHQiI%2BCjxmZUZsb29kIGZsb29kLW9wYWNpdHk9IjAiIHJlc3VsdD0iQmFja2dyb3VuZEltYWdlRml4Ii8%2BCjxmZUNvbG9yTWF0cml4IGluPSJTb3VyY2VBbHBoYSIgdHlwZT0ibWF0cml4IiB2YWx1ZXM9IjAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDEyNyAwIi8%2BCjxmZU9mZnNldC8%2BCjxmZUdhdXNzaWFuQmx1ciBzdGREZXZpYXRpb249IjQuMTY2NjciLz4KPGZlQ29sb3JNYXRyaXggdHlwZT0ibWF0cml4IiB2YWx1ZXM9IjAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAuMjUgMCIvPgo8ZmVCbGVuZCBtb2RlPSJvdmVybGF5IiBpbjI9IkJhY2tncm91bmRJbWFnZUZpeCIgcmVzdWx0PSJlZmZlY3QxX2Ryb3BTaGFkb3ciLz4KPGZlQmxlbmQgbW9kZT0ibm9ybWFsIiBpbj0iU291cmNlR3JhcGhpYyIgaW4yPSJlZmZlY3QxX2Ryb3BTaGFkb3ciIHJlc3VsdD0ic2hhcGUiLz4KPC9maWx0ZXI%2BCjxsaW5lYXJHcmFkaWVudCBpZD0icGFpbnQwX2xpbmVhciIgeDE9IjQ5LjkzOTIiIHkxPSIwLjI1NzgxMiIgeDI9IjQ5LjkzOTIiIHkyPSI5OS43NDIzIiBncmFkaWVudFVuaXRzPSJ1c2VyU3BhY2VPblVzZSI%2BCjxzdG9wIHN0b3AtY29sb3I9IndoaXRlIi8%2BCjxzdG9wIG9mZnNldD0iMSIgc3RvcC1jb2xvcj0id2hpdGUiIHN0b3Atb3BhY2l0eT0iMCIvPgo8L2xpbmVhckdyYWRpZW50Pgo8L2RlZnM%2BCjwvc3ZnPgo%3D)](https://code.visualstudio.com/)
[![Build with PlatformIO](https://img.shields.io/badge/build%20with-PlatformIO-orange?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMjUwMCIgaGVpZ2h0PSIyNTAwIiB2aWV3Qm94PSIwIDAgMjU2IDI1NiIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiBwcmVzZXJ2ZUFzcGVjdFJhdGlvPSJ4TWlkWU1pZCI+PHBhdGggZD0iTTEyOCAwQzkzLjgxIDAgNjEuNjY2IDEzLjMxNCAzNy40OSAzNy40OSAxMy4zMTQgNjEuNjY2IDAgOTMuODEgMCAxMjhjMCAzNC4xOSAxMy4zMTQgNjYuMzM0IDM3LjQ5IDkwLjUxQzYxLjY2NiAyNDIuNjg2IDkzLjgxIDI1NiAxMjggMjU2YzM0LjE5IDAgNjYuMzM0LTEzLjMxNCA5MC41MS0zNy40OUMyNDIuNjg2IDE5NC4zMzQgMjU2IDE2Mi4xOSAyNTYgMTI4YzAtMzQuMTktMTMuMzE0LTY2LjMzNC0zNy40OS05MC41MUMxOTQuMzM0IDEzLjMxNCAxNjIuMTkgMCAxMjggMCIgZmlsbD0iI0ZGN0YwMCIvPjxwYXRoIGQ9Ik0yNDkuMzg2IDEyOGMwIDY3LjA0LTU0LjM0NyAxMjEuMzg2LTEyMS4zODYgMTIxLjM4NkM2MC45NiAyNDkuMzg2IDYuNjEzIDE5NS4wNCA2LjYxMyAxMjggNi42MTMgNjAuOTYgNjAuOTYgNi42MTQgMTI4IDYuNjE0YzY3LjA0IDAgMTIxLjM4NiA1NC4zNDYgMTIxLjM4NiAxMjEuMzg2IiBmaWxsPSIjRkZGIi8+PHBhdGggZD0iTTE2MC44NjkgNzQuMDYybDUuMTQ1LTE4LjUzN2M1LjI2NC0uNDcgOS4zOTItNC44ODYgOS4zOTItMTAuMjczIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzJzLTEwLjMyIDQuNjItMTAuMzIgMTAuMzJjMCAzLjc1NSAyLjAxMyA3LjAzIDUuMDEgOC44MzdsLTUuMDUgMTguMTk1Yy0xNC40MzctMy42Ny0yNi42MjUtMy4zOS0yNi42MjUtMy4zOWwtMi4yNTggMS4wMXYxNDAuODcybDIuMjU4Ljc1M2MxMy42MTQgMCA3My4xNzctNDEuMTMzIDczLjMyMy04NS4yNyAwLTMxLjYyNC0yMS4wMjMtNDUuODI1LTQwLjU1NS01Mi4xOTd6TTE0Ni41MyAxNjQuOGMtMTEuNjE3LTE4LjU1Ny02LjcwNi02MS43NTEgMjMuNjQzLTY3LjkyNSA4LjMyLTEuMzMzIDE4LjUwOSA0LjEzNCAyMS41MSAxNi4yNzkgNy41ODIgMjUuNzY2LTM3LjAxNSA2MS44NDUtNDUuMTUzIDUxLjY0NnptMTguMjE2LTM5Ljc1MmE5LjM5OSA5LjM5OSAwIDAgMC05LjM5OSA5LjM5OSA5LjM5OSA5LjM5OSAwIDAgMCA5LjQgOS4zOTkgOS4zOTkgOS4zOTkgMCAwIDAgOS4zOTgtOS40IDkuMzk5IDkuMzk5IDAgMCAwLTkuMzk5LTkuMzk4em0yLjgxIDguNjcyYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDkgMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OXoiIGZpbGw9IiNFNTcyMDAiLz48cGF0aCBkPSJNMTAxLjM3MSA3Mi43MDlsLTUuMDIzLTE4LjkwMWMyLjg3NC0xLjgzMiA0Ljc4Ni01LjA0IDQuNzg2LTguNzAxIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzItNS42OTkgMC0xMC4zMTkgNC42Mi0xMC4zMTkgMTAuMzIgMCA1LjY4MiA0LjU5MiAxMC4yODkgMTAuMjY3IDEwLjMxN0w5NS44IDc0LjM3OGMtMTkuNjA5IDYuNTEtNDAuODg1IDIwLjc0Mi00MC44ODUgNTEuODguNDM2IDQ1LjAxIDU5LjU3MiA4NS4yNjcgNzMuMTg2IDg1LjI2N1Y2OC44OTJzLTEyLjI1Mi0uMDYyLTI2LjcyOSAzLjgxN3ptMTAuMzk1IDkyLjA5Yy04LjEzOCAxMC4yLTUyLjczNS0yNS44OC00NS4xNTQtNTEuNjQ1IDMuMDAyLTEyLjE0NSAxMy4xOS0xNy42MTIgMjEuNTExLTE2LjI4IDMwLjM1IDYuMTc1IDM1LjI2IDQ5LjM2OSAyMy42NDMgNjcuOTI2em0tMTguODItMzkuNDZhOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTkgOS4zOTggOS4zOTkgOS4zOTkgMCAwIDAgOS40IDkuNCA5LjM5OSA5LjM5OSAwIDAgMCA5LjM5OC05LjQgOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTktOS4zOTl6bS0yLjgxIDguNjcxYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDggMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OHoiIGZpbGw9IiNGRjdGMDAiLz48L3N2Zz4=)](https://platformio.org/)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/k0i05/library/esp_ds18b20.svg)](https://registry.platformio.org/libraries/k0i05/esp_ds18b20)
[![ESP Component Registry](https://components.espressif.com/components/k0i05/esp_ds18b20/badge.svg)](https://components.espressif.com/components/k0i05/esp_ds18b20)

This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the Maxim-Integrated DS18B20 1-wire temperature sensor.  Information on features and functionality are documented and can be found in the `ds18b20.h` header file and in the `documentation` folder.

## Repository

The component is hosted on github and is located here: <https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/peripherals/owb/esp_ds18b20>

## General Usage

To get started, simply copy the component to your project's `components` folder and reference the `ds18b20.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

```text
components
└── esp_ds18b20
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── ds18b20_version.h
    │   └── ds18b20.h
    └── ds18b20.c
```

## Basic Example

Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings, prints DS18S20 sensors detected on the 1-wire bus, and makes a measurement request from the sensor at a user defined interval and prints the results.

```c
#include <ds18b20.h>

void owb0_ds18b20_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t                   last_wake_time = xTaskGetTickCount ();
    //
    // initialize owb device configuration
    ds18b20_config_t             dev_cfg = OWB_DS18B20_CONFIG_DEFAULT;
    ds18b20_handle_t             dev_hdl;
    onewire_device_iter_handle_t dev_iter_hdl;
    onewire_device_t             dev;
    // owb ds18b20 device detection
    onewire_device_t             devs[5];
    uint8_t                      devs_size = sizeof(devs) / sizeof(devs[0]);
    uint8_t                      devs_count = 0;
    //
    // detect ds18b20 devices on 1-wire bus
    esp_err_t ret = ds18b20_detect(owb0_bus_hdl, devs, devs_size, &devs_count);
    if(ret == ESP_OK) {
        ESP_LOGW(APP_TAG, "ds18b20 devices detected: %u", devs_count);
        for(uint8_t i = 0; i < devs_count; i++) {
            ESP_LOGI(APP_TAG, "ds18b20(%u), address: %016llX", i, devs[i].address);
        }
    } else {
        ESP_LOGE(APP_TAG, "ds18b20 device detect failed (%s)", esp_err_to_name(ret));
    }
    //
    // instantiate 1-wire device iterator handle
    ESP_ERROR_CHECK( onewire_new_device_iter(owb0_bus_hdl, &dev_iter_hdl) );
    //
    // get 1-wire device - assumes there is only one ds18b20 device on the bus
    if (onewire_device_iter_get_next(dev_iter_hdl, &dev) == ESP_OK) { // found a new device, let's check if we can upgrade it to a DS18B20
        // check if the device is a ds18b20, if so, return the ds18b20 handle
        if (ds18b20_init(&dev, &dev_cfg, &dev_hdl) == ESP_OK) {
            ESP_LOGI(APP_TAG, "found a ds18b20, address: %016llX", dev.address);
        } else {
            ESP_LOGI(APP_TAG, "found an unknown device, address: %016llX", dev.address);
            assert(dev.address);
        }
    }
    //
    // free device iter handle
    ESP_ERROR_CHECK( onewire_del_device_iter(dev_iter_hdl) );
    //
    // get/set/get resolution
    ds18b20_resolutions_t dev_reso;
    ESP_ERROR_CHECK( ds18b20_get_resolution(dev_hdl, &dev_reso) );
    ESP_LOGW(APP_TAG, "ds18b20 get resolution: %d", dev_reso);
    ESP_ERROR_CHECK( ds18b20_set_resolution(dev_hdl, DS18B20_RESOLUTION_11BIT) );
    ESP_ERROR_CHECK( ds18b20_get_resolution(dev_hdl, &dev_reso) );
    ESP_LOGW(APP_TAG, "ds18b20 get resolution: %d", dev_reso);
    //
    // validate device handle
    if(dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "ds18b20 handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for( ;; ) {
        ESP_LOGI(APP_TAG, "######################## DS18B20 - START #########################");
        //
        // handle sensor
        float temperature;
        esp_err_t result = ds18b20_get_measurement(dev_hdl, &temperature);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "ds18b20 device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "temperature:     %.2f°C", temperature);
        }
        //
        ESP_LOGI(APP_TAG, "######################## DS18B20 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, OWB0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    ds18b20_delete( dev_hdl );
    vTaskDelete( NULL );
}
```

Copyright (c) 2024 Eric Gionet (<gionet.c.eric@gmail.com>)
