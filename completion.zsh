_ros_service_caller() {
    local state

    _arguments \
        {-h,--help}'[show help]' \
        {-f,--file}'[load yaml file]: :->file' \
        '1: :->service'

    case $state in
        file)
            _path_files -g '*.yaml'
            ;;
        service)
            compadd $(rosservice list 2>/dev/null)
            ;;
    esac
}
compdef _ros_service_caller ros-service-caller
