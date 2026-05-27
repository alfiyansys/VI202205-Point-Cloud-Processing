#!/bin/bash
set -e
cd "$(dirname "$0")"

if [ ! -d ".venv" ]; then
    echo "Membuat virtual environment..."
    python3 -m venv .venv
fi

if ! .venv/bin/python -c "import open3d, numpy, matplotlib, sklearn, notebook" 2>/dev/null; then
    echo "Menginstall dependencies..."
    .venv/bin/pip install --upgrade pip -q
    .venv/bin/pip install -r requirements.txt -q
fi

echo "Menjalankan Jupyter Notebook..."
.venv/bin/jupyter notebook point_cloud_segmentation.ipynb
