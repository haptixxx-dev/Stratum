# Stratum C4 Architecture Documentation Generator
#
# Usage:
#   make all      - Generate diagrams and PDF
#   make diagrams - Generate only PlantUML diagrams
#   make pdf      - Generate only PDF (requires diagrams first)
#   make clean    - Remove generated files
#   make docker   - Generate using Docker (no local deps needed)

.PHONY: all diagrams pdf clean docker help

PLANTUML_VERSION := 1.2024.8
PLANTUML_JAR_URL := https://github.com/plantuml/plantuml/releases/download/v$(PLANTUML_VERSION)/plantuml-$(PLANTUML_VERSION).jar

# Directories
BUILD_DIR := build
IMAGE_DIR := images/c4

# Source files
PUML_SRC := c4-architecture.puml
MD_SRC := $(BUILD_DIR)/c4-architecture-full.md
PDF_OUT := $(BUILD_DIR)/stratum-c4-architecture.pdf

# PlantUML diagrams to generate
DIAGRAMS := C4_SystemContext C4_Container C4_Component_CoreLib C4_Component_Editor \
            C4_Component_Scene C4_Component_OSM C4_Component_ProcGen C4_Component_Renderer \
            C4_Dynamic_AppLifecycle C4_Dynamic_OSMImport C4_Dynamic_FrameRender C4_Deployment

DIAGRAM_PNGS := $(addprefix $(IMAGE_DIR)/,$(addsuffix .png,$(DIAGRAMS)))

help:
	@echo "Stratum C4 Architecture Documentation Generator"
	@echo ""
	@echo "Usage:"
	@echo "  make all      - Generate diagrams and PDF"
	@echo "  make diagrams - Generate only PlantUML diagrams"
	@echo "  make pdf      - Generate only PDF (requires diagrams first)"
	@echo "  make clean    - Remove generated files"
	@echo "  make docker   - Generate using Docker (no local deps needed)"
	@echo ""
	@echo "Dependencies:"
	@echo "  - plantuml (pacman -S plantuml)"
	@echo "  - pandoc (pacman -S pandoc)"
	@echo "  - texlive (pacman -S texlive)"

all: diagrams pdf
	@echo "✓ Documentation generated: $(PDF_OUT)"

$(BUILD_DIR) $(IMAGE_DIR):
	mkdir -p $@

# Generate all PlantUML diagrams
diagrams: $(IMAGE_DIR) $(PUML_SRC)
	@echo "Generating PlantUML diagrams..."
	plantuml -tpng -Sdpi=150 $(PUML_SRC) -o $(IMAGE_DIR)/
	@echo "✓ Diagrams generated in $(IMAGE_DIR)/"

# Generate the combined markdown document
$(MD_SRC): $(BUILD_DIR)
	@echo "Creating combined markdown..."
	./generate-pdf.sh

# Generate PDF from markdown
pdf: $(BUILD_DIR) diagrams
	@echo "Generating PDF..."
	./generate-pdf.sh
	@echo "✓ PDF generated: $(PDF_OUT)"

# Docker-based generation (no local dependencies needed)
docker: $(BUILD_DIR) $(IMAGE_DIR)
	@echo "Generating with Docker..."
	docker run --rm -v "$(PWD):/docs" \
		plantuml/plantuml:latest \
		-tpng -Sdpi=150 /docs/$(PUML_SRC) -o /docs/$(IMAGE_DIR)/
	docker run --rm -v "$(PWD):/docs" \
		pandoc/latex:latest \
		/docs/$(BUILD_DIR)/c4-architecture-full.md \
		-o /docs/$(PDF_OUT) \
		--toc --toc-depth=3 \
		-V colorlinks=true -V linkcolor=blue
	@echo "✓ PDF generated: $(PDF_OUT)"

# Clean generated files
clean:
	rm -rf $(BUILD_DIR)
	rm -rf $(IMAGE_DIR)
	@echo "✓ Cleaned generated files"

# Install dependencies (Arch Linux)
install-deps:
	sudo pacman -S --needed plantuml pandoc texlive-latexextra
