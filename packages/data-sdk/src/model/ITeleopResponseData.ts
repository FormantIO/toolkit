export interface ITeleopResponseData {
    state: "inProgress" | "requestAssistance" | "success" | "failure";
    notes?: string;
}
