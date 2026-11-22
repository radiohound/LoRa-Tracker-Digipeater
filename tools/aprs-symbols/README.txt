APRS symbols (primary, alternate and overlay-character sheets, 24 px at 2x) from the aprs.fi symbol set by Heikki Hannikainen, OH7LZB:
https://github.com/hessu/aprs-symbols  -- "released to the APRS community ... for any APRS application, for free" (its README, 2015-12-03).
The set's COPYRIGHT.md says the licensing differs per symbol (mostly redrawn versions of the standard WA8LMF bitmap set; a few brand logos belong to their owners).
These are NOT covered by this repository's MIT license: they stay under the terms of the original set.

Used by tools/setup.html to show the symbol the chosen table and code make.
Layout of each sheet: 16 columns x 6 rows of 48 px cells; the cell of a character c is index ord(c) - 33
(column = index % 16, row = index // 16). Sheet 0 is the primary table (/), sheet 1 the alternate table (\), sheet 2 the overlay characters (0-9, A-Z).
