# Vishay VEML7700 Sensor

[![K0I05](https://img.shields.io/badge/K0I05-a9a9a9?logo=data:image/svg%2bxml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIxODgiIGhlaWdodD0iMTg3Ij48cGF0aCBmaWxsPSIjNDU0QjU0IiBkPSJNMTU1LjU1NSAyMS45M2MxOS4yNzMgMTUuOTggMjkuNDcyIDM5LjM0NSAzMi4xNjggNjMuNzg5IDEuOTM3IDIyLjkxOC00LjU1MyA0Ni42Ni0xOC44NDggNjQuNzgxQTUwOS40NzggNTA5LjQ3OCAwIDAgMSAxNjUgMTU1bC0xLjQ4NCAxLjg4M2MtMTMuMTk2IDE2LjUzMS0zNS41NTUgMjcuMjE1LTU2LjMzOSAyOS45MDItMjguMzEyIDIuOC01Mi4yNTUtNC43MzctNzQuNzMyLTIxLjcxNUMxMy4xNzIgMTQ5LjA5IDIuOTczIDEyNS43MjUuMjc3IDEwMS4yODEtMS42NiA3OC4zNjMgNC44MyA1NC42MjEgMTkuMTI1IDM2LjVBNTA5LjQ3OCA1MDkuNDc4IDAgMCAxIDIzIDMybDEuNDg0LTEuODgzQzM3LjY4IDEzLjU4NiA2MC4wNCAyLjkwMiA4MC44MjMuMjE1YzI4LjMxMi0yLjggNTIuMjU1IDQuNzM3IDc0LjczMiAyMS43MTVaIi8+PHBhdGggZmlsbD0iI0ZERkRGRCIgZD0iTTExOS44NjcgNDUuMjdDMTI4LjkzMiA1Mi4yNiAxMzMuODIgNjMgMTM2IDc0Yy42MyA0Ljk3Mi44NDIgOS45NTMuOTUzIDE0Ljk2LjA0NCAxLjkxMS4xMjIgMy44MjIuMjAzIDUuNzMxLjM0IDEyLjIxLjM0IDEyLjIxLTMuMTU2IDE3LjMwOWE5NS42MDQgOTUuNjA0IDAgMCAxLTQuMTg4IDMuNjI1Yy00LjUgMy43MTctNi45NzQgNy42ODgtOS43MTcgMTIuODAzQzEwNi45NCAxNTIuNzkyIDEwNi45NCAxNTIuNzkyIDk3IDE1N2MtMy40MjMuNTkyLTUuODAxLjY4NS04Ljg3OS0xLjA3NC05LjgyNi03Ljg4LTE2LjAzNi0xOS41OS0yMS44NTgtMzAuNTEyLTIuNTM0LTQuNTc1LTUuMDA2LTcuMjEtOS40NjYtMTAuMDItMy43MTQtMi44ODItNS40NS02Ljk4Ni02Ljc5Ny0xMS4zOTQtLjU1LTQuODg5LS41NjEtOS4zMTYgMS0xNCAuMDkzLTEuNzYzLjE4Mi0zLjUyNy4yMzktNS4yOTIuNDkxLTEzLjg4NCAzLjg2Ni0yNy4wNTcgMTQuMTU2LTM3LjAyOCAxNy4yMTgtMTQuMzM2IDM1Ljg1OC0xNS4wNjYgNTQuNDcyLTIuNDFaIi8+PHBhdGggZmlsbD0iI0M2RDVFMCIgZD0iTTEwOSAzOWMxMS43MDMgNS4yNTUgMTkuMjA2IDEzLjE4NiAyNC4yOTMgMjUuMDA0IDIuODU3IDguMjQgMy40NyAxNi4zMTYgMy42NiAyNC45NTYuMDQ0IDEuOTExLjEyMiAzLjgyMi4yMDMgNS43MzEuMzQgMTIuMjEuMzQgMTIuMjEtMy4xNTYgMTcuMzA5YTk1LjYwNCA5NS42MDQgMCAwIDEtNC4xODggMy42MjVjLTQuNSAzLjcxNy02Ljk3NCA3LjY4OC05LjcxNyAxMi44MDNDMTA2LjgwNCAxNTMuMDQxIDEwNi44MDQgMTUzLjA0MSA5NyAxNTdjLTIuMzMyLjA3OC00LjY2OC4wOS03IDBsMi4xMjUtMS44NzVjNS40My01LjQ0NSA4Ljc0NC0xMi41NzcgMTEuNzU0LTE5LjU1OWEzNDkuNzc1IDM0OS43NzUgMCAwIDEgNC40OTYtOS44NzlsMS42NDgtMy41NWMyLjI0LTMuNTU1IDQuNDEtNC45OTYgNy45NzctNy4xMzcgMi4zMjMtMi42MSAyLjMyMy0yLjYxIDQtNWwtMyAxYy0yLjY4LjE0OC01LjMxOS4yMy04IC4yNWwtMi4xOTUuMDYzYy01LjI4Ny4wMzktNS4yODcuMDM5LTcuNzc4LTEuNjUzLTEuNjY2LTIuNjkyLTEuNDUzLTQuNTYtMS4wMjctNy42NiAyLjM5NS00LjM2MiA0LjkyNC04LjA0IDkuODI4LTkuNTcgMi4zNjQtLjQ2OCA0LjUxNC0uNTI4IDYuOTIyLS40OTNsMi40MjIuMDI4TDEyMSA5MmwtMS0yYTkyLjc1OCA5Mi43NTggMCAwIDEtLjM2LTQuNTg2QzExOC42IDY5LjYzMiAxMTYuNTE3IDU2LjA5NCAxMDQgNDVjLTUuOTA0LTQuNjY0LTExLjYtNi4wODgtMTktNyA3LjU5NC00LjI2NCAxNi4yMjMtMS44MSAyNCAxWiIvPjxwYXRoIGZpbGw9IiM0OTUwNTgiIGQ9Ik03NyA5MmM0LjYxMyAxLjY3MSA3LjI2IDMuOTQ1IDEwLjA2MyA3LjkzOCAxLjA3OCAzLjUyMy45NzYgNS41NDYtLjA2MyA5LjA2Mi0yLjk4NCAyLjk4NC02LjI1NiAyLjM2OC0xMC4yNSAyLjM3NWwtMi4yNzcuMDc0Yy01LjI5OC4wMjgtOC4yNTQtLjk4My0xMi40NzMtNC40NDktMi44MjYtMy41OTctMi40MTYtNy42MzQtMi0xMiA0LjUwMi00LjcyOCAxMC45OS0zLjc2IDE3LTNaIi8+PHBhdGggZmlsbD0iIzQ4NEY1NyIgZD0ibTExOCA5MS43NSAzLjEyNS0uMDc4YzMuMjU0LjM3MSA0LjU5NyAxLjAwMiA2Ljg3NSAzLjMyOC42MzkgNC4yMzEuMjkgNi40NDItMS42ODggMTAuMjUtMy40MjggNC4wNzgtNS44MjcgNS41OTgtMTEuMTk1IDYuMTQ4LTEuNDE0LjAwOC0yLjgyOCAwLTQuMjQyLS4wMjNsLTIuMTY4LjAzNWMtMi45OTgtLjAxNy01LjE1Ny0uMDMzLTcuNjcyLTEuNzU4LTEuNjgxLTIuNjg0LTEuNDYtNC41NTItMS4wMzUtNy42NTIgMi4zNzUtNC4zMjUgNC44OTQtOC4wMDkgOS43NS05LjU1OSAyLjc3Ny0uNTQ0IDUuNDItLjY0OSA4LjI1LS42OTFaIi8+PHBhdGggZmlsbD0iIzUyNTg2MCIgZD0iTTg2IDEzNGgxNmwxIDRjLTIgMi0yIDItNS4xODggMi4yNjZMOTQgMTQwLjI1bC0zLjgxMy4wMTZDODcgMTQwIDg3IDE0MCA4NSAxMzhsMS00WiIvPjwvc3ZnPg==)](https://github.com/K0I05)
[![License: MIT](https://cdn.prod.website-files.com/5e0f1144930a8bc8aace526c/65dd9eb5aaca434fac4f1c34_License-MIT-blue.svg)](/LICENSE)
[![Language](https://img.shields.io/badge/Language-C-navy.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Framework](https://img.shields.io/badge/Framework-ESP_IDF-red.svg)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)
[![Edited with VS Code](https://img.shields.io/badge/Edited_with-VS_Code-blue?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIxMDAiIGhlaWdodD0iMTAwIiBmaWxsPSJub25lIj48bWFzayBpZD0iYSIgd2lkdGg9IjEwMCIgaGVpZ2h0PSIxMDAiIHg9IjAiIHk9IjAiIG1hc2stdHlwZT0iYWxwaGEiIG1hc2tVbml0cz0idXNlclNwYWNlT25Vc2UiPjxwYXRoIGZpbGw9IiNmZmYiIGZpbGwtcnVsZT0iZXZlbm9kZCIgZD0iTTcwLjkxMiA5OS4zMTdhNi4yMjMgNi4yMjMgMCAwIDAgNC45Ni0uMTlsMjAuNTg5LTkuOTA3QTYuMjUgNi4yNSAwIDAgMCAxMDAgODMuNTg3VjE2LjQxM2E2LjI1IDYuMjUgMCAwIDAtMy41NC01LjYzMkw3NS44NzQuODc0YTYuMjI2IDYuMjI2IDAgMCAwLTcuMTA0IDEuMjFMMjkuMzU1IDM4LjA0IDEyLjE4NyAyNS4wMWE0LjE2MiA0LjE2MiAwIDAgMC01LjMxOC4yMzZsLTUuNTA2IDUuMDA5YTQuMTY4IDQuMTY4IDAgMCAwLS4wMDQgNi4xNjJMMTYuMjQ3IDUwIDEuMzYgNjMuNTgzYTQuMTY4IDQuMTY4IDAgMCAwIC4wMDQgNi4xNjJsNS41MDYgNS4wMWE0LjE2MiA0LjE2MiAwIDAgMCA1LjMxOC4yMzZsMTcuMTY4LTEzLjAzMkw2OC43NyA5Ny45MTdhNi4yMTcgNi4yMTcgMCAwIDAgMi4xNDMgMS40Wk03NS4wMTUgMjcuMyA0NS4xMSA1MGwyOS45MDYgMjIuNzAxVjI3LjNaIiBjbGlwLXJ1bGU9ImV2ZW5vZGQiLz48L21hc2s%2BPGcgbWFzaz0idXJsKCNhKSI%2BPHBhdGggZmlsbD0iIzAwNjVBOSIgZD0iTTk2LjQ2MSAxMC43OTYgNzUuODU3Ljg3NmE2LjIzIDYuMjMgMCAwIDAtNy4xMDcgMS4yMDdsLTY3LjQ1MSA2MS41YTQuMTY3IDQuMTY3IDAgMCAwIC4wMDQgNi4xNjJsNS41MSA1LjAwOWE0LjE2NyA0LjE2NyAwIDAgMCA1LjMyLjIzNmw4MS4yMjgtNjEuNjJjMi43MjUtMi4wNjcgNi42MzktLjEyNCA2LjYzOSAzLjI5N3YtLjI0YTYuMjUgNi4yNSAwIDAgMC0zLjUzOS01LjYzWiIvPjxnIGZpbHRlcj0idXJsKCNiKSI%2BPHBhdGggZmlsbD0iIzAwN0FDQyIgZD0ibTk2LjQ2MSA4OS4yMDQtMjAuNjA0IDkuOTJhNi4yMjkgNi4yMjkgMCAwIDEtNy4xMDctMS4yMDdsLTY3LjQ1MS02MS41YTQuMTY3IDQuMTY3IDAgMCAxIC4wMDQtNi4xNjJsNS41MS01LjAwOWE0LjE2NyA0LjE2NyAwIDAgMSA1LjMyLS4yMzZsODEuMjI4IDYxLjYyYzIuNzI1IDIuMDY3IDYuNjM5LjEyNCA2LjYzOS0zLjI5N3YuMjRhNi4yNSA2LjI1IDAgMCAxLTMuNTM5IDUuNjNaIi8%2BPC9nPjxnIGZpbHRlcj0idXJsKCNjKSI%2BPHBhdGggZmlsbD0iIzFGOUNGMCIgZD0iTTc1Ljg1OCA5OS4xMjZhNi4yMzIgNi4yMzIgMCAwIDEtNy4xMDgtMS4yMWMyLjMwNiAyLjMwNyA2LjI1LjY3NCA2LjI1LTIuNTg4VjQuNjcyYzAtMy4yNjItMy45NDQtNC44OTUtNi4yNS0yLjU4OWE2LjIzMiA2LjIzMiAwIDAgMSA3LjEwOC0xLjIxbDIwLjYgOS45MDhBNi4yNSA2LjI1IDAgMCAxIDEwMCAxNi40MTN2NjcuMTc0YTYuMjUgNi4yNSAwIDAgMS0zLjU0MSA1LjYzM2wtMjAuNjAxIDkuOTA2WiIvPjwvZz48cGF0aCBmaWxsPSJ1cmwoI2QpIiBmaWxsLXJ1bGU9ImV2ZW5vZGQiIGQ9Ik03MC44NTEgOTkuMzE3YTYuMjI0IDYuMjI0IDAgMCAwIDQuOTYtLjE5TDk2LjQgODkuMjJhNi4yNSA2LjI1IDAgMCAwIDMuNTQtNS42MzNWMTYuNDEzYTYuMjUgNi4yNSAwIDAgMC0zLjU0LTUuNjMyTDc1LjgxMi44NzRhNi4yMjYgNi4yMjYgMCAwIDAtNy4xMDQgMS4yMUwyOS4yOTQgMzguMDQgMTIuMTI2IDI1LjAxYTQuMTYyIDQuMTYyIDAgMCAwLTUuMzE3LjIzNmwtNS41MDcgNS4wMDlhNC4xNjggNC4xNjggMCAwIDAtLjAwNCA2LjE2MkwxNi4xODYgNTAgMS4yOTggNjMuNTgzYTQuMTY4IDQuMTY4IDAgMCAwIC4wMDQgNi4xNjJsNS41MDcgNS4wMDlhNC4xNjIgNC4xNjIgMCAwIDAgNS4zMTcuMjM2TDI5LjI5NCA2MS45NmwzOS40MTQgMzUuOTU4YTYuMjE4IDYuMjE4IDAgMCAwIDIuMTQzIDEuNFpNNzQuOTU0IDI3LjMgNDUuMDQ4IDUwbDI5LjkwNiAyMi43MDFWMjcuM1oiIGNsaXAtcnVsZT0iZXZlbm9kZCIgb3BhY2l0eT0iLjI1IiBzdHlsZT0ibWl4LWJsZW5kLW1vZGU6b3ZlcmxheSIvPjwvZz48ZGVmcz48ZmlsdGVyIGlkPSJiIiB3aWR0aD0iMTE2LjcyNyIgaGVpZ2h0PSI5Mi4yNDYiIHg9Ii04LjM5NCIgeT0iMTUuODI5IiBjb2xvci1pbnRlcnBvbGF0aW9uLWZpbHRlcnM9InNSR0IiIGZpbHRlclVuaXRzPSJ1c2VyU3BhY2VPblVzZSI%2BPGZlRmxvb2QgZmxvb2Qtb3BhY2l0eT0iMCIgcmVzdWx0PSJCYWNrZ3JvdW5kSW1hZ2VGaXgiLz48ZmVDb2xvck1hdHJpeCBpbj0iU291cmNlQWxwaGEiIHZhbHVlcz0iMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMTI3IDAiLz48ZmVPZmZzZXQvPjxmZUdhdXNzaWFuQmx1ciBzdGREZXZpYXRpb249IjQuMTY3Ii8%2BPGZlQ29sb3JNYXRyaXggdmFsdWVzPSIwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwLjI1IDAiLz48ZmVCbGVuZCBpbjI9IkJhY2tncm91bmRJbWFnZUZpeCIgbW9kZT0ib3ZlcmxheSIgcmVzdWx0PSJlZmZlY3QxX2Ryb3BTaGFkb3ciLz48ZmVCbGVuZCBpbj0iU291cmNlR3JhcGhpYyIgaW4yPSJlZmZlY3QxX2Ryb3BTaGFkb3ciIHJlc3VsdD0ic2hhcGUiLz48L2ZpbHRlcj48ZmlsdGVyIGlkPSJjIiB3aWR0aD0iNDcuOTE3IiBoZWlnaHQ9IjExNi4xNTEiIHg9IjYwLjQxNyIgeT0iLTguMDc2IiBjb2xvci1pbnRlcnBvbGF0aW9uLWZpbHRlcnM9InNSR0IiIGZpbHRlclVuaXRzPSJ1c2VyU3BhY2VPblVzZSI%2BPGZlRmxvb2QgZmxvb2Qtb3BhY2l0eT0iMCIgcmVzdWx0PSJCYWNrZ3JvdW5kSW1hZ2VGaXgiLz48ZmVDb2xvck1hdHJpeCBpbj0iU291cmNlQWxwaGEiIHZhbHVlcz0iMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMTI3IDAiLz48ZmVPZmZzZXQvPjxmZUdhdXNzaWFuQmx1ciBzdGREZXZpYXRpb249IjQuMTY3Ii8%2BPGZlQ29sb3JNYXRyaXggdmFsdWVzPSIwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwIDAgMCAwLjI1IDAiLz48ZmVCbGVuZCBpbjI9IkJhY2tncm91bmRJbWFnZUZpeCIgbW9kZT0ib3ZlcmxheSIgcmVzdWx0PSJlZmZlY3QxX2Ryb3BTaGFkb3ciLz48ZmVCbGVuZCBpbj0iU291cmNlR3JhcGhpYyIgaW4yPSJlZmZlY3QxX2Ryb3BTaGFkb3ciIHJlc3VsdD0ic2hhcGUiLz48L2ZpbHRlcj48bGluZWFyR3JhZGllbnQgaWQ9ImQiIHgxPSI0OS45MzkiIHgyPSI0OS45MzkiIHkxPSIuMjU4IiB5Mj0iOTkuNzQyIiBncmFkaWVudFVuaXRzPSJ1c2VyU3BhY2VPblVzZSI%2BPHN0b3Agc3RvcC1jb2xvcj0iI2ZmZiIvPjxzdG9wIG9mZnNldD0iMSIgc3RvcC1jb2xvcj0iI2ZmZiIgc3RvcC1vcGFjaXR5PSIwIi8%2BPC9saW5lYXJHcmFkaWVudD48L2RlZnM%2BPC9zdmc%2B&logoSize=auto)](https://code.visualstudio.com/)
[![Build with PlatformIO](https://img.shields.io/badge/build%20with-PlatformIO-orange?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMjUwMCIgaGVpZ2h0PSIyNTAwIiB2aWV3Qm94PSIwIDAgMjU2IDI1NiIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiBwcmVzZXJ2ZUFzcGVjdFJhdGlvPSJ4TWlkWU1pZCI+PHBhdGggZD0iTTEyOCAwQzkzLjgxIDAgNjEuNjY2IDEzLjMxNCAzNy40OSAzNy40OSAxMy4zMTQgNjEuNjY2IDAgOTMuODEgMCAxMjhjMCAzNC4xOSAxMy4zMTQgNjYuMzM0IDM3LjQ5IDkwLjUxQzYxLjY2NiAyNDIuNjg2IDkzLjgxIDI1NiAxMjggMjU2YzM0LjE5IDAgNjYuMzM0LTEzLjMxNCA5MC41MS0zNy40OUMyNDIuNjg2IDE5NC4zMzQgMjU2IDE2Mi4xOSAyNTYgMTI4YzAtMzQuMTktMTMuMzE0LTY2LjMzNC0zNy40OS05MC41MUMxOTQuMzM0IDEzLjMxNCAxNjIuMTkgMCAxMjggMCIgZmlsbD0iI0ZGN0YwMCIvPjxwYXRoIGQ9Ik0yNDkuMzg2IDEyOGMwIDY3LjA0LTU0LjM0NyAxMjEuMzg2LTEyMS4zODYgMTIxLjM4NkM2MC45NiAyNDkuMzg2IDYuNjEzIDE5NS4wNCA2LjYxMyAxMjggNi42MTMgNjAuOTYgNjAuOTYgNi42MTQgMTI4IDYuNjE0YzY3LjA0IDAgMTIxLjM4NiA1NC4zNDYgMTIxLjM4NiAxMjEuMzg2IiBmaWxsPSIjRkZGIi8+PHBhdGggZD0iTTE2MC44NjkgNzQuMDYybDUuMTQ1LTE4LjUzN2M1LjI2NC0uNDcgOS4zOTItNC44ODYgOS4zOTItMTAuMjczIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzJzLTEwLjMyIDQuNjItMTAuMzIgMTAuMzJjMCAzLjc1NSAyLjAxMyA3LjAzIDUuMDEgOC44MzdsLTUuMDUgMTguMTk1Yy0xNC40MzctMy42Ny0yNi42MjUtMy4zOS0yNi42MjUtMy4zOWwtMi4yNTggMS4wMXYxNDAuODcybDIuMjU4Ljc1M2MxMy42MTQgMCA3My4xNzctNDEuMTMzIDczLjMyMy04NS4yNyAwLTMxLjYyNC0yMS4wMjMtNDUuODI1LTQwLjU1NS01Mi4xOTd6TTE0Ni41MyAxNjQuOGMtMTEuNjE3LTE4LjU1Ny02LjcwNi02MS43NTEgMjMuNjQzLTY3LjkyNSA4LjMyLTEuMzMzIDE4LjUwOSA0LjEzNCAyMS41MSAxNi4yNzkgNy41ODIgMjUuNzY2LTM3LjAxNSA2MS44NDUtNDUuMTUzIDUxLjY0NnptMTguMjE2LTM5Ljc1MmE5LjM5OSA5LjM5OSAwIDAgMC05LjM5OSA5LjM5OSA5LjM5OSA5LjM5OSAwIDAgMCA5LjQgOS4zOTkgOS4zOTkgOS4zOTkgMCAwIDAgOS4zOTgtOS40IDkuMzk5IDkuMzk5IDAgMCAwLTkuMzk5LTkuMzk4em0yLjgxIDguNjcyYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDkgMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OXoiIGZpbGw9IiNFNTcyMDAiLz48cGF0aCBkPSJNMTAxLjM3MSA3Mi43MDlsLTUuMDIzLTE4LjkwMWMyLjg3NC0xLjgzMiA0Ljc4Ni01LjA0IDQuNzg2LTguNzAxIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzItNS42OTkgMC0xMC4zMTkgNC42Mi0xMC4zMTkgMTAuMzIgMCA1LjY4MiA0LjU5MiAxMC4yODkgMTAuMjY3IDEwLjMxN0w5NS44IDc0LjM3OGMtMTkuNjA5IDYuNTEtNDAuODg1IDIwLjc0Mi00MC44ODUgNTEuODguNDM2IDQ1LjAxIDU5LjU3MiA4NS4yNjcgNzMuMTg2IDg1LjI2N1Y2OC44OTJzLTEyLjI1Mi0uMDYyLTI2LjcyOSAzLjgxN3ptMTAuMzk1IDkyLjA5Yy04LjEzOCAxMC4yLTUyLjczNS0yNS44OC00NS4xNTQtNTEuNjQ1IDMuMDAyLTEyLjE0NSAxMy4xOS0xNy42MTIgMjEuNTExLTE2LjI4IDMwLjM1IDYuMTc1IDM1LjI2IDQ5LjM2OSAyMy42NDMgNjcuOTI2em0tMTguODItMzkuNDZhOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTkgOS4zOTggOS4zOTkgOS4zOTkgMCAwIDAgOS40IDkuNCA5LjM5OSA5LjM5OSAwIDAgMCA5LjM5OC05LjQgOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTktOS4zOTl6bS0yLjgxIDguNjcxYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDggMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OHoiIGZpbGw9IiNGRjdGMDAiLz48L3N2Zz4=)](https://platformio.org/)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/k0i05/library/esp_veml7700.svg)](https://registry.platformio.org/libraries/k0i05/esp_veml7700)
[![ESP Component Registry](https://components.espressif.com/components/k0i05/esp_veml7700/badge.svg)](https://components.espressif.com/components/k0i05/esp_veml7700)

This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the Vishay VEML7700 I2C sensor.  Information on features and functionality are documented and can be found in the `veml7700.h` header file and in the `documentation` folder.

## Repository

The component is hosted on github and is located here: <https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/peripherals/i2c/esp_veml7700>

## General Usage

To get started, simply copy the component to your project's `components` folder and reference the `veml7700.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

The auto-calibrate algorithms aren't consistent and are still being worked on.

```text
components
└── esp_veml7700
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── veml7700_version.h
    │   └── veml7700.h
    └── veml7700.c
```

## Typical Luminance Values

Luminance ranges with use-case examples:

* `10-5 lx`: Light from Sirius, the brightest star in the night sky
* `10-4 lx`: Total starlight, overcast sky
* `0.002 lx`: Moonless clear night sky with airglow
* `0.01 lx`: Quarter moon, 0.27 lx; full moon on a clear night
* `1 lx`: Full moon overhead at tropical latitudes
* `3.4 lx`: Dark limit of civil twilight under a clear sky
* `50 lx`: Family living room
* `80 lx`: Hallway / bathroom
* `100 lx`: Very dark overcast day
* `320 lx to 500 lx`: Office lighting
* `400 lx`: Sunrise or sunset on a clear day
* `1,000 lx`: Overcast day; typical TV studio lighting
* `10,000 lx to 25,000 lx`: Full daylight (not direct sun)
* `32,000 lx to 130,000 lx`: Direct sunlight

## Basic Example

Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings and makes a measurement request from the sensor at user defined interval and prints the results.

```c
#include <veml7700.h>

void i2c0_veml7700_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time  = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    veml7700_config_t dev_cfg       = I2C_VEML7700_CONFIG_DEFAULT;
    veml7700_handle_t dev_hdl;
    //
    // init device
    veml7700_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "veml7700 handle init failed");
        assert(dev_hdl);
    }
    //
    // optimize sensor
    //veml7700_optimize_configuration(dev_hdl);
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## VEML7700 - START #########################");
        //
        // handle sensor
        float ambient_light;
        //uint16_t als_counts;
        esp_err_t result = veml7700_get_ambient_light(dev_hdl, &ambient_light);
        //esp_err_t result = veml7700_get_ambient_light_counts(dev_hdl, &als_counts);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "veml7700 device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "ambient light:     %.2f lux", ambient_light);
            //ESP_LOGI(APP_TAG, "ambient light:     %u counts", als_counts);
        }
        //
        ESP_LOGI(APP_TAG, "######################## VEML7700 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    veml7700_delete( dev_hdl );
    vTaskDelete( NULL );
}
```

Copyright (c) 2024 Eric Gionet (<gionet.c.eric@gmail.com>)
