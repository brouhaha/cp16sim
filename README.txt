Microcode-Level Simulator for Western Digital MCP1600
Copyright 2016 Eric Smith <spacewar@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License version 3
as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.


cp16sim is a microcode-level simulator for the Western Digital MCP1600
chipset, which is used in:

* Digital Equipment Corporation LSI-11 (PDP-11/03)
* Western Digital WD16 chipset (used in Alpha Micro AM-100)
* Western Digital WD9000 Pascal Microengine chipset

cp16sim currently simulates the WD9000. Support for the LSI-11 and WD16
will be added when the PLAs of the control stores of those versions of
the chipset have been transcribed.

As of 2016-02-04, cp16sim is incomplete, especially regarding I/O, and
there are known to be serious bugs. It works well enough to execute
the first few dozen p-code instructions of the ACD PDQ-3 boot ROM
before going into the weeds.
