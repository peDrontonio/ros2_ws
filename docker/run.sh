#!/bin/bash

# Script de conveniência para gerenciar containers Docker do projeto PRM
# Uso: ./run.sh [ubuntu20|ubuntu22] [comando]

set -e

# Cores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Função para mostrar ajuda
show_help() {
    echo -e "${BLUE}🤖 Script de Gerenciamento Docker - Projeto PRM${NC}"
    echo ""
    echo "Uso: $0 [VERSÃO] [COMANDO]"
    echo ""
    echo -e "${YELLOW}Versões disponíveis:${NC}"
    echo "  ubuntu20    Ubuntu 20.04 + ROS2 Galactic"
    echo "  ubuntu22    Ubuntu 22.04 + ROS2 Humble"
    echo ""
    echo -e "${YELLOW}Comandos:${NC}"
    echo "  build       Construir imagem Docker"
    echo "  run         Executar container interativo"
    echo "  start       Iniciar container em background"
    echo "  stop        Parar container"
    echo "  shell       Entrar no container em execução"
    echo "  logs        Mostrar logs do container"
    echo "  clean       Limpar containers e imagens"
    echo "  status      Mostrar status dos containers"
    echo ""
    echo -e "${YELLOW}Exemplos:${NC}"
    echo "  $0 ubuntu22 run          # Executar Ubuntu 22.04 interativo"
    echo "  $0 ubuntu20 build        # Construir imagem Ubuntu 20.04"
    echo "  $0 ubuntu22 shell        # Entrar no container Ubuntu 22.04"
    echo ""
}

# Verificar se docker está instalado
check_docker() {
    if ! command -v docker &> /dev/null; then
        echo -e "${RED}❌ Docker não está instalado!${NC}"
        echo "Instale o Docker primeiro: https://docs.docker.com/get-docker/"
        exit 1
    fi
    
    # Verificar Docker Compose (plugin ou standalone)
    if docker compose version &> /dev/null; then
        DOCKER_COMPOSE="docker compose"
        echo -e "${GREEN}✅ Docker Compose (plugin) detectado${NC}"
    elif command -v docker-compose &> /dev/null; then
        DOCKER_COMPOSE="docker-compose"
        echo -e "${GREEN}✅ Docker Compose (standalone) detectado${NC}"
    else
        echo -e "${RED}❌ Docker Compose não está instalado!${NC}"
        echo "Instale o Docker Compose primeiro"
        exit 1
    fi
}

# Configurar X11 para interface gráfica
setup_x11() {
    if [ "$DISPLAY" ]; then
        echo -e "${BLUE}🖥️  Configurando X11 para interface gráfica...${NC}"
        xhost +local:docker > /dev/null 2>&1 || true
    fi
}

# Verificar argumentos
if [ $# -lt 1 ]; then
    show_help
    exit 1
fi

VERSION=$1
COMMAND=${2:-run}

# Validar versão
case $VERSION in
    ubuntu20|20)
        PROFILE="ubuntu20"
        SERVICE="prm-ubuntu20"
        VERSION_NAME="Ubuntu 20.04 + ROS2 Galactic"
        ;;
    ubuntu22|22)
        PROFILE="ubuntu22"
        SERVICE="prm-ubuntu22"
        VERSION_NAME="Ubuntu 22.04 + ROS2 Humble"
        ;;
    help|-h|--help)
        show_help
        exit 0
        ;;
    *)
        echo -e "${RED}❌ Versão inválida: $VERSION${NC}"
        echo "Use 'ubuntu20' ou 'ubuntu22'"
        exit 1
        ;;
esac

# Verificar dependências
check_docker

# Navegar para o diretório correto
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo -e "${BLUE}🚀 Gerenciando container: $VERSION_NAME${NC}"

# Executar comando
case $COMMAND in
    build)
        echo -e "${YELLOW}🔨 Construindo imagem...${NC}"
        $DOCKER_COMPOSE --profile $PROFILE build --no-cache
        echo -e "${GREEN}✅ Imagem construída com sucesso!${NC}"
        ;;
    
    run)
        setup_x11
        echo -e "${YELLOW}🏃 Executando container interativo...${NC}"
        $DOCKER_COMPOSE --profile $PROFILE up --build -d
        $DOCKER_COMPOSE --profile $PROFILE exec $SERVICE bash
        ;;
    
    start)
        setup_x11
        echo -e "${YELLOW}▶️  Iniciando container em background...${NC}"
        $DOCKER_COMPOSE --profile $PROFILE up --build -d
        echo -e "${GREEN}✅ Container iniciado! Use '$0 $VERSION shell' para entrar.${NC}"
        ;;
    
    stop)
        echo -e "${YELLOW}⏹️  Parando container...${NC}"
        $DOCKER_COMPOSE --profile $PROFILE down
        echo -e "${GREEN}✅ Container parado!${NC}"
        ;;
    
    shell)
        if ! $DOCKER_COMPOSE --profile $PROFILE ps | grep -q "Up"; then
            echo -e "${YELLOW}📦 Container não está executando. Iniciando...${NC}"
            setup_x11
            $DOCKER_COMPOSE --profile $PROFILE up --build -d
        fi
        echo -e "${YELLOW}🐚 Entrando no container...${NC}"
        $DOCKER_COMPOSE --profile $PROFILE exec $SERVICE bash
        ;;
    
    logs)
        echo -e "${YELLOW}📋 Mostrando logs do container...${NC}"
        $DOCKER_COMPOSE --profile $PROFILE logs -f
        ;;
    
    clean)
        echo -e "${YELLOW}🧹 Limpando containers e imagens...${NC}"
        $DOCKER_COMPOSE --profile $PROFILE down -v --rmi all
        echo -e "${GREEN}✅ Limpeza concluída!${NC}"
        ;;
    
    status)
        echo -e "${YELLOW}📊 Status dos containers:${NC}"
        $DOCKER_COMPOSE ps
        echo ""
        echo -e "${YELLOW}📈 Uso de recursos:${NC}"
        docker stats --no-stream --format "table {{.Container}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.NetIO}}"
        ;;
    
    *)
        echo -e "${RED}❌ Comando inválido: $COMMAND${NC}"
        show_help
        exit 1
        ;;
esac
