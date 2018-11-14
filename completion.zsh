_ros_service_caller() {
    local state

    _arguments \
        {-h,--help}'[show help]' \
        {-f,--file}'[load yaml file]: :->file' \
        '1: :->commands' \
        '2: :->args'

    case $state in
        file)
            _path_files -g '*.yaml'
            ;;
        commands)
            compadd 'service' 'topic'
            ;;
        args)
            case $words[2] in
                service)
                    compadd $(rosservice list 2>/dev/null)
                    ;;
                topic)
                    compadd $(rostopic list 2>/dev/null)
                    ;;
            esac
            ;;
    esac
}
compdef _ros_service_caller ros-service-caller
