#!/bin/bash

pandoc report.md \
        --pdf-engine=xelatex \
        --variable geometry:margin=1.8cm \
        --variable fontsize=10.5pt \
        --variable mainfont="DejaVu Serif" \
        --variable sansfont="DejaVu Sans" \
        --variable monofont="DejaVu Sans Mono" \
        --variable lang=id \
        --highlight-style=tango \
        -V "header-includes=\\usepackage{graphicx}\\usepackage{float}\\floatplacement{figure}{H}\\setkeys{Gin}{width=0.55\\linewidth}" \
        -o report.pdf