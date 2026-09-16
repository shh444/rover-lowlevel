#!/usr/bin/env bash
# 로봇(192.168.5.2)으로 가는 네트워크 인터페이스를 찾아 CycloneDDS 설정을 만들고 CYCLONEDDS_URI 를 내보낸다.
# SDK 의 setup_cyclonedds_env.sh 와 같은 방식이며, 파일을 /tmp 대신 이 폴더에 둔다 (컨테이너/호스트 공용).
#   source docker/dds_env.sh            (호스트에서도, docker/sdk.sh 로 들어간 컨테이너 안에서도 동작)
#   source docker/dds_env.sh enP2p1s0   (인터페이스를 직접 지정)
ROBOT_IP=${ROBOT_IP:-192.168.5.2}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IFACE=${1:-$(ip route get "$ROBOT_IP" 2>/dev/null | grep -oP 'dev \K\S+')}
if [ -z "$IFACE" ]; then
    echo "[dds_env] $ROBOT_IP 로 가는 인터페이스를 찾지 못했습니다. PC 를 192.168.5.xxx/24 로 설정하거나 인터페이스 이름을 인자로 주세요." >&2
    return 1 2>/dev/null || exit 1
fi
XML="$HERE/cyclonedds.generated.xml"
cat > "$XML" <<XMLEOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain Id="0">
        <General>
            <Interfaces>
                <NetworkInterface name="$IFACE" priority="default" multicast="true" />
            </Interfaces>
            <AllowMulticast>spdp</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
            <DontRoute>true</DontRoute>
        </General>
        <Discovery>
            <EnableTopicDiscoveryEndpoints>true</EnableTopicDiscoveryEndpoints>
        </Discovery>
        <Internal>
            <Watermarks>
                <WhcHigh>500kB</WhcHigh>
            </Watermarks>
        </Internal>
    </Domain>
</CycloneDDS>
XMLEOF
export CYCLONEDDS_URI="file://$XML"
echo "[dds_env] interface=$IFACE  CYCLONEDDS_URI=$CYCLONEDDS_URI"
