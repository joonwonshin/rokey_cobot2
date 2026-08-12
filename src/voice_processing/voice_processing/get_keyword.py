# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"

import os
import rclpy
import pyaudio
import yaml
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate  # d2 이거를 langchain_core로 바꿈
# from langchain.chains import LLMChain

from std_srvs.srv import Trigger
from voice_processing.MicController import MicController, MicConfig

from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT

############ Package Path & Environment Setting ############

#----------------------------------------------------------------
# current_dir = os.getcwd()
# package_path = get_package_share_directory("pick_and_place_voice")

# env_path = "/home/rokey/cobot_ws/src/cobot2_ws/pick_and_place_voice/resource/.env"
# load_dotenv(dotenv_path=env_path)
# is_load = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
# openai_api_key = os.getenv("OPENAI_API_KEY")
#-----------------------------------------------------------------

PACKAGE_NAME = "voice_processing"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
RESOURCE_PATH = os.path.join(PACKAGE_PATH, "resource")
ENV_PATH = os.path.join(RESOURCE_PATH, ".env")
load_dotenv(dotenv_path=ENV_PATH)
openai_api_key = os.getenv("OPENAI_API_KEY")


def _default_objects_path() -> str:
    """ws 루트 `config/objects.yaml` 경로. `COBOT2_OBJECTS` 로 덮어쓴다.

    경로 규칙은 `graspx.launch.py:default_objects_file()` /
    `vla_command.launch.py:default_allowed_classes()` 와 같다 — 셋이 어긋나면
    "이 노드만 다른 물체를 찾는다"는 사고가 나므로 상대경로 계산도 그대로 맞춘다
    (`share/voice_processing` 에서 위로 4단계 = ws 루트).
    """
    path = os.environ.get('COBOT2_OBJECTS')
    if path:
        return path
    return os.path.abspath(os.path.join(PACKAGE_PATH, '..', '..', '..', '..',
                                        'config', 'objects.yaml'))


def load_detect_names(path: str) -> list:
    """`objects.yaml` 의 `detect:` 이름 목록. YOLO(`yolo_seg_node`)가 실제로 찾는 대상과

    같은 정본이다 — 이 프롬프트의 어휘가 여기서 벗어나면 LLM 이 추출한 이름을 브리지가
    영원히 못 찾는다(2026-08-09 지적: 예전엔 hammer/screwdriver/wrench 로 하드코딩돼
    있어서 이 ws 의 YOLO 클래스와 애초에 안 맞았다). 못 읽으면 **조용히 넘어가지 않는다** —
    빈 리스트를 돌려주고 프롬프트에 그 사실을 그대로 적어 사람이 눈치채게 한다.
    """
    try:
        with open(path, encoding='utf-8') as f:
            doc = yaml.safe_load(f) or {}
        return [str(n).strip() for n in (doc.get('detect') or []) if str(n).strip()]
    except (OSError, yaml.YAMLError) as exc:
        print(f'[get_keyword] {path} 를 못 읽었다 ({exc}) — 프롬프트 어휘가 비어 있다')
        return []


OBJECTS_PATH = _default_objects_path()
DETECT_NAMES = load_detect_names(OBJECTS_PATH)

############ AI Processor ############
# class AIProcessor:
#     def __init__(self):



############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):

        print(PACKAGE_PATH, RESOURCE_PATH, ENV_PATH)

        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )

        # 🔴 이 리스트는 하드코딩하지 않는다 — `config/objects.yaml`(ws 루트, YOLO
        # `yolo_seg_node`·VLA 경로 `vla_command_node` 와 같은 정본)의 `detect:` 를 그대로
        # 쓴다. 여기서 벗어난 이름을 LLM 이 뽑으면 grasp_bridge_node 가 "해당하는 물체가
        # 없다"로만 실패하고 원인은 안 보인다(2026-08-09 지적 — 예전엔 hammer/screwdriver/
        # wrench 로 하드코딩돼 있어 이 ws 의 YOLO 클래스와 애초에 안 맞았다).
        if DETECT_NAMES:
            tool_list_line = ', '.join(DETECT_NAMES)
        else:
            tool_list_line = (f'(비어 있음 — {OBJECTS_PATH} 를 못 읽었다. '
                              'COBOT2_OBJECTS 나 ws 경로를 확인할 것)')
        example_a, example_b = (DETECT_NAMES + ['apple', 'orange'])[:2]

        prompt_content = f"""
            당신은 사용자의 문장에서 특정 물체와 목적지를 추출해야 합니다.

            <목표>
            - 문장에서 다음 리스트에 포함된 물체를 최대한 정확히 추출하세요.
            - 문장에 등장하는 물체의 목적지(어디로 옮기라고 했는지)도 함께 추출하세요.

            <물체 리스트> (config/objects.yaml 의 detect — YOLO 가 실제로 찾는 이름과 같다)
            - {tool_list_line}

            <목적지>
            - pos1, pos2, pos3 (자유 형식 — 위 물체 리스트와는 다른 종류다)

            <출력 형식>
            - 다음 형식을 반드시 따르세요: [물체1 물체2 ... / pos1 pos2 ...]
            - 물체와 위치는 각각 공백으로 구분
            - 물체가 없으면 앞쪽은 공백 없이 비우고, 목적지가 없으면 '/' 뒤는 공백 없이 비웁니다.
            - 물체와 목적지의 순서는 등장 순서를 따릅니다.

            <특수 규칙>
            - 명확한 이름이 없지만 문맥상 유추 가능한 경우(예: "마시는 그릇" → {example_b})는 리스트 내 항목으로 최대한 추론해 반환하세요.
            - 리스트에 없는 물체는 절대 만들어 내지 마세요 — 리스트 밖 이름을 반환하면 로봇이 찾지 못합니다.
            - 다수의 물체와 목적지가 동시에 등장할 경우 각각에 대해 정확히 매칭하여 순서대로 출력하세요.

            <예시>
            - 입력: "{example_a}를 pos1에 가져다 놔"
            출력: {example_a} / pos1

            - 입력: "왼쪽에 있는 {example_a}와 {example_b}를 pos1에 넣어줘"
            출력: {example_a} {example_b} / pos1

            - 입력: "왼쪽에 있는 {example_a}를 줘"
            출력: {example_a} /

            - 입력: "{example_a}는 pos2에 두고 {example_b}는 pos1에 둬"
            출력: {example_a} {example_b} / pos2 pos1

            <사용자 입력>
            "{{user_input}}"
        """

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = self.prompt_template | self.llm
        # self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)
        self.stt = STT(openai_api_key=openai_api_key)


        super().__init__("get_keyword_node")
        # 오디오 설정
        mic_config = MicConfig(
            chunk=12000,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            device_index=10,
            buffer_size=24000,
        )
        self.mic_controller = MicController(config=mic_config)
        # self.ai_processor = AIProcessor()

        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("wait for client's request...")
        self.get_keyword_srv = self.create_service(
            Trigger, "get_keyword", self.get_keyword
        )
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

    def extract_keyword(self, output_message):  # d2 이 함수 일부 수정함
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response.content

        object, target = result.strip().split("/")

        object = object.split()
        target = target.split()

        print(f"llm's response: {object}")
        print(f"object: {object}")
        print(f"target: {target}")
        return object
    
    def get_keyword(self, request, response):  # 요청과 응답 객체를 받아야 함    # d2 이 함수 일부 수정함
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            return None

        while not self.wakeup_word.is_wakeup():
            pass

        # STT --> Keword Extract --> Embedding
        output_message = self.stt.speech2text()
        keyword = self.extract_keyword(output_message)

        self.get_logger().warn(f"Detected tools: {keyword}")

        # 응답 객체 설정
        response.success = True
        response.message = " ".join(keyword)  # 감지된 키워드를 응답 메시지로 반환
        return response


def main():  # d2 메인문 일부 수정
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
